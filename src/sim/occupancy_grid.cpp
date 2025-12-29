#include "occupancy_grid.hpp"
#include <cassert>
#include <fstream>

OccupancyGrid::OccupancyGrid(int width, int height, double resolution, Vec2 const& origin)
    : width_(width),
    height_(height),
    resolution_(resolution),
    origin_(origin),
    data_(width * height, 0)
    {
        assert(width_ > 0 && height_ > 0);
    }

// i row, j col, 0-index
int OccupancyGrid::index(int i, int j) const noexcept {
    return i * width_ + j;
}

std::pair<int, int> OccupancyGrid::coord(int ind) const {
    return {ind / width_, ind % width_};
}

bool OccupancyGrid::isOccupied(int i, int j) const {
    return data_[index(i, j)] != 0;
}

void OccupancyGrid::setOccupied(int i, int j, bool occ) {
    data_[index(i, j)] = occ ? 1 : 0;
}

std::pair<int, int> OccupancyGrid::worldToGrid(Vec2 const& p) const {
    int j = static_cast<int>(std::floor((p.x - origin_.x) / resolution_));
    int i = static_cast<int>(std::floor((p.y - origin_.y) / resolution_));
    return {i, j};
}

Vec2 OccupancyGrid::gridToWorld(int i, int j) const {
    return Vec2{
        origin_.x + (j + 0.5) * resolution_,
        origin_.y + (i + 0.5) * resolution_
    };
}

void OccupancyGrid::addRectangle(Vec2 center, double w, double h) {
    for (int i = 0; i < height_; i++) {
        for (int j = 0; j < width_; j++) {
            Vec2 cell = gridToWorld(i, j);
            if (cell.x < center.x + w / 2 && cell.x > center.x - w / 2
            && cell.y < center.y + h / 2 && cell.y > center.y - h / 2) {
                setOccupied(i, j, true);
            }
        }
    }
}
void OccupancyGrid::addCircle(Vec2 center, double radius) {
    for (int i = 0; i < height_; i++) {
        for (int j = 0; j < width_; j++) {
            if (center.distance(gridToWorld(i, j)) < radius){
                setOccupied(i, j, true);
            }
        }
    }
}

void OccupancyGrid::writeSVG(char const* filename) const {
    std::ofstream out(filename);
    out << "<svg xmlns=\"http://www.w3.org/2000/svg\" "
        << "width=\"" << width_ * 10 << "\" "
        << "height=\"" << height_ * 10 << "\">\n";

    for (int i = 0; i < height_; i++) {
        for (int j = 0; j < width_; j++) {
            const std::string fill = isOccupied(i, j) ? "black" : "white";

            out << "<rect x=\"" << j * 10
                << "\" y=\"" << i * 10
                << "\" width=\"10\" height=\"10\" "
                << "fill=\"" << fill << "\" />\n";
        }
    }

    out << "</svg>\n";
}