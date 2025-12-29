#include "simulator.hpp"
#include <cmath>
#include <fstream>
#include <sstream>
#include <iomanip>

Simulator::Simulator(
    OccupancyGrid& grid,
    Robot& robot,
    AStarPlanner& planner,
    Vec2 goal,
    double wheelbase
)
    : grid_(grid),
    robot_(robot),
    planner_(planner),
    controller_(wheelbase),
    goal_(goal) {}

void Simulator::run(double dt, int max_steps, int frame_interval) {
    auto start = robot_.state().position;
    auto path = planner_.plan(grid_, start, goal_);
    if (path.empty()) return;

    trajectory_.clear();
    trajectory_.push_back(start);

    size_t path_idx = 0;

    for (int i = 0; i < max_steps; ++i) {
        auto readings = robot_.sense(grid_);

        Vec2 pos = robot_.state().position;
        trajectory_.push_back(pos);
        
        bool near_goal = (path_idx + 1 == path.size());
        double waypoint_tol = 0.3 * grid_.resolution();
        if (pos.distance(path[path_idx]) < waypoint_tol && !near_goal) {
            path_idx++;
        }
        Vec2 waypoint = path[path_idx];
        if (near_goal) {
            waypoint = goal_;
        }

        auto [vl, vr] = controller_.computeControl(robot_.state(), waypoint);
        robot_.step(vl, vr, dt);

        if (i % frame_interval == 0) {
            std::ostringstream oss;
            oss << "frame_" << std::setfill('0') << std::setw(4) << i << ".svg";
            writeFrameSVG(oss.str());
        }

        if (reachedGoal()) {
            break;
        }
    }

    writeSummarySVG("summary.svg");
}

bool Simulator::reachedGoal() const {
    return robot_.state().position.distance(goal_) < 0.05;
}

// svg visualizations all by cursor
namespace {
    struct Bounds {
        double min_x, min_y, max_x, max_y;
    };

    Bounds computeBounds(
        OccupancyGrid const& grid,
        std::vector<Vec2> const& trajectory,
        std::vector<Vec2> const& planned_path,
        Vec2 const& goal,
        RobotState const& state,
        std::vector<Reading> const& readings
    ) {
        Bounds b;
        b.min_x = grid.origin().x;
        b.min_y = grid.origin().y;
        b.max_x = grid.origin().x + grid.width() * grid.resolution();
        b.max_y = grid.origin().y + grid.height() * grid.resolution();

        for (auto const& p : trajectory) {
            b.min_x = std::min(b.min_x, p.x);
            b.min_y = std::min(b.min_y, p.y);
            b.max_x = std::max(b.max_x, p.x);
            b.max_y = std::max(b.max_y, p.y);
        }

        for (auto const& p : planned_path) {
            b.min_x = std::min(b.min_x, p.x);
            b.min_y = std::min(b.min_y, p.y);
            b.max_x = std::max(b.max_x, p.x);
            b.max_y = std::max(b.max_y, p.y);
        }

        b.min_x = std::min(b.min_x, goal.x);
        b.min_y = std::min(b.min_y, goal.y);
        b.max_x = std::max(b.max_x, goal.x);
        b.max_y = std::max(b.max_y, goal.y);

        b.min_x = std::min(b.min_x, state.position.x);
        b.min_y = std::min(b.min_y, state.position.y);
        b.max_x = std::max(b.max_x, state.position.x);
        b.max_y = std::max(b.max_y, state.position.y);

        for (auto const& reading : readings) {
            double world_angle = state.theta + reading.angle;
            Vec2 end = state.position + Vec2{std::cos(world_angle), std::sin(world_angle)} * reading.range;
            b.min_x = std::min(b.min_x, end.x);
            b.min_y = std::min(b.min_y, end.y);
            b.max_x = std::max(b.max_x, end.x);
            b.max_y = std::max(b.max_y, end.y);
        }

        double padding = std::max((b.max_x - b.min_x), (b.max_y - b.min_y)) * 0.1;
        b.min_x -= padding;
        b.min_y -= padding;
        b.max_x += padding;
        b.max_y += padding;

        return b;
    }
}

void Simulator::writeFrameSVG(std::string const& filename) const {
    std::ofstream out(filename);
    if (!out) return;

    auto readings = robot_.sense(grid_);
    auto state = robot_.state();
    Bounds bounds = computeBounds(grid_, trajectory_, planned_path_, goal_, state, readings);

    double view_width = bounds.max_x - bounds.min_x;
    double view_height = bounds.max_y - bounds.min_y;
    double scale = 100.0;

    out << "<svg xmlns=\"http://www.w3.org/2000/svg\" "
        << "width=\"" << view_width * scale << "\" "
        << "height=\"" << view_height * scale << "\" "
        << "viewBox=\"" << bounds.min_x << " " << bounds.min_y << " "
        << view_width << " " << view_height << "\">\n";

    // Grid cells
    for (int i = 0; i < grid_.height(); ++i) {
        for (int j = 0; j < grid_.width(); ++j) {
            Vec2 cell = grid_.gridToWorld(i, j);
            double cell_size = grid_.resolution();
            std::string fill = grid_.isOccupied(i, j) ? "#333" : "#fff";
            out << "<rect x=\"" << (cell.x - cell_size/2) << "\" "
                << "y=\"" << (cell.y - cell_size/2) << "\" "
                << "width=\"" << cell_size << "\" "
                << "height=\"" << cell_size << "\" "
                << "fill=\"" << fill << "\" stroke=\"#ccc\" stroke-width=\"0.01\"/>\n";
        }
    }

    // Planned path
    if (!planned_path_.empty()) {
        out << "<polyline points=\"";
        for (size_t i = 0; i < planned_path_.size(); ++i) {
            out << planned_path_[i].x << "," << planned_path_[i].y;
            if (i < planned_path_.size() - 1) out << " ";
        }
        out << "\" fill=\"none\" stroke=\"#00f\" stroke-width=\"0.05\" opacity=\"0.6\"/>\n";
    }

    // Trajectory
    if (trajectory_.size() > 1) {
        out << "<polyline points=\"";
        for (size_t i = 0; i < trajectory_.size(); ++i) {
            out << trajectory_[i].x << "," << trajectory_[i].y;
            if (i < trajectory_.size() - 1) out << " ";
        }
        out << "\" fill=\"none\" stroke=\"#0f0\" stroke-width=\"0.03\" opacity=\"0.8\"/>\n";
    }

    // Sensor rays
    for (auto const& reading : readings) {
        double world_angle = state.theta + reading.angle;
        Vec2 end = state.position + Vec2{std::cos(world_angle), std::sin(world_angle)} * reading.range;
        out << "<line x1=\"" << state.position.x << "\" y1=\"" << state.position.y << "\" "
            << "x2=\"" << end.x << "\" y2=\"" << end.y << "\" "
            << "stroke=\"#f00\" stroke-width=\"0.02\" opacity=\"0.5\"/>\n";
    }

    // Robot pose (triangle)
    double robot_size = 0.1;
    double dx = std::cos(state.theta) * robot_size;
    double dy = std::sin(state.theta) * robot_size;
    double perp_x = -std::sin(state.theta) * robot_size * 0.5;
    double perp_y = std::cos(state.theta) * robot_size * 0.5;
    Vec2 front = state.position + Vec2{dx, dy};
    Vec2 left = state.position + Vec2{perp_x, perp_y};
    Vec2 right = state.position - Vec2{perp_x, perp_y};
    out << "<polygon points=\""
        << front.x << "," << front.y << " "
        << left.x << "," << left.y << " "
        << right.x << "," << right.y << "\" "
        << "fill=\"#ff0\" stroke=\"#000\" stroke-width=\"0.02\"/>\n";

    // Goal
    out << "<circle cx=\"" << goal_.x << "\" cy=\"" << goal_.y << "\" "
        << "r=\"0.05\" fill=\"#0f0\" stroke=\"#000\" stroke-width=\"0.02\"/>\n";

    out << "</svg>\n";
}

void Simulator::writeSummarySVG(std::string const& filename) const {
    std::ofstream out(filename);
    if (!out) return;

    auto s = robot_.state();
    std::vector<Reading> empty_readings;
    Bounds bounds = computeBounds(grid_, trajectory_, planned_path_, goal_, s, empty_readings);

    double view_width = bounds.max_x - bounds.min_x;
    double view_height = bounds.max_y - bounds.min_y;

    out << "<svg xmlns=\"http://www.w3.org/2000/svg\" "
        << "width=\"" << view_width * 100 << "\" "
        << "height=\"" << view_height * 100 << "\" "
        << "viewBox=\"" << bounds.min_x << " " << bounds.min_y << " "
        << view_width << " " << view_height << "\">\n";

    // Grid cells
    for (int i = 0; i < grid_.height(); ++i) {
        for (int j = 0; j < grid_.width(); ++j) {
            Vec2 cell = grid_.gridToWorld(i, j);
            double cell_size = grid_.resolution();
            std::string fill = grid_.isOccupied(i, j) ? "#333" : "#fff";
            out << "<rect x=\"" << (cell.x - cell_size/2) << "\" "
                << "y=\"" << (cell.y - cell_size/2) << "\" "
                << "width=\"" << cell_size << "\" "
                << "height=\"" << cell_size << "\" "
                << "fill=\"" << fill << "\" stroke=\"#ccc\" stroke-width=\"0.01\"/>\n";
        }
    }

    // Planned path
    if (!planned_path_.empty()) {
        out << "<polyline points=\"";
        for (size_t i = 0; i < planned_path_.size(); ++i) {
            out << planned_path_[i].x << "," << planned_path_[i].y;
            if (i < planned_path_.size() - 1) out << " ";
        }
        out << "\" fill=\"none\" stroke=\"#00f\" stroke-width=\"0.05\" opacity=\"0.6\"/>\n";
    }

    // Full trajectory
    if (trajectory_.size() > 1) {
        out << "<polyline points=\"";
        for (size_t i = 0; i < trajectory_.size(); ++i) {
            out << trajectory_[i].x << "," << trajectory_[i].y;
            if (i < trajectory_.size() - 1) out << " ";
        }
        out << "\" fill=\"none\" stroke=\"#0f0\" stroke-width=\"0.03\" opacity=\"0.8\"/>\n";
    }

    // Final robot pose
    auto state = robot_.state();
    double robot_size = 0.1;
    double dx = std::cos(state.theta) * robot_size;
    double dy = std::sin(state.theta) * robot_size;
    double perp_x = -std::sin(state.theta) * robot_size * 0.5;
    double perp_y = std::cos(state.theta) * robot_size * 0.5;
    Vec2 front = state.position + Vec2{dx, dy};
    Vec2 left = state.position + Vec2{perp_x, perp_y};
    Vec2 right = state.position - Vec2{perp_x, perp_y};
    out << "<polygon points=\""
        << front.x << "," << front.y << " "
        << left.x << "," << left.y << " "
        << right.x << "," << right.y << "\" "
        << "fill=\"#ff0\" stroke=\"#000\" stroke-width=\"0.02\"/>\n";

    // Start position
    if (!trajectory_.empty()) {
        out << "<circle cx=\"" << trajectory_[0].x << "\" cy=\"" << trajectory_[0].y << "\" "
            << "r=\"0.08\" fill=\"#0ff\" stroke=\"#000\" stroke-width=\"0.02\"/>\n";
    }

    // Goal
    out << "<circle cx=\"" << goal_.x << "\" cy=\"" << goal_.y << "\" "
        << "r=\"0.05\" fill=\"#0f0\" stroke=\"#000\" stroke-width=\"0.02\"/>\n";

    out << "</svg>\n";
}