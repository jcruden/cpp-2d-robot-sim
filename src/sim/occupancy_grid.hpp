#pragma once
#include <vector>
#include <cstdint>
#include <utility>
#include "../util/math.hpp"

class OccupancyGrid {
public:
    OccupancyGrid(int width, int height, double resolution);

    bool isOccupied(int i, int j) const;
    void setOccupied(int i, int j, bool occ = true);

    int width() const noexcept { return width_; };
    int height() const noexcept { return height_; };
    
    std::pair<int, int> worldToGrid(Vec2 const& p) const;
    int index(int i, int j) const noexcept;
    std::pair<int, int> coord(int ind) const;

private:
    int width_;
    int height_;
    double resolution_;
    std::vector<uint8_t> data_; // 0 = free, 1 = occupied
};