#include "occupancy_grid.hpp"
#include <cassert>

OccupancyGrid::OccupancyGrid(int width, int height, double resolution)
    : width_(width),
    height_(height),
    resolution_(resolution),
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
    int j = static_cast<int>(std::floor(p.x / resolution_));
    int i = static_cast<int>(std::floor(p.y / resolution_));
    return {i, j};
}