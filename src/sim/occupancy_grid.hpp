#pragma once
#include <vector>
#include <cstdint>
#include <utility>
#include "../util/math.hpp"

class OccupancyGrid {
public:
    OccupancyGrid(int width, int height, double resolution, Vec2 origin = {0.0, 0.0});

    bool isOccupied(int i, int j) const;
    void setOccupied(int i, int j, bool occ = true);

    int width() const noexcept { return width_; };
    int height() const noexcept { return height_; };
    double resolution() const noexcept { return resolution_; };
    Vec2 origin() const noexcept { return origin_; };
    
    // conversions
    std::pair<int, int> worldToGrid(Vec2 const& p) const;
    Vec2 gridToWorld(int i, int j) const;

    // obstacles
    void addRectangle(Vec2 center, double w, double h);
    void addCircle(Vec2 center, double radius);

    void writeSVG(char const* filename) const;

    int index(int i, int j) const noexcept;
    std::pair<int, int> coord(int ind) const;
    
private:
    int width_;
    int height_;
    double resolution_;
    Vec2 origin_;
    std::vector<uint8_t> data_; // 0 = free, 1 = occupied
};