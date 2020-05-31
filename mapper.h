#pragma once

#include "pose.h"
#include "tile.h"
#include "tiles_storage.h"

#include <gdal.h>
#include <gdal_priv.h>

#include <opencv2/opencv.hpp>

class Mapper {
public:

    Mapper();

    ~Mapper();

    void feed(const Pose &pose, cv::Mat &frame);

private:
    // Tile -> priority
    // priority decreases by one on each feed if the tile is not used
    // once it reaches zero Tile is removed from tiles
    TilesStorage tiles;

    void getInternalCoords(const Pose &pose, double dx, double dy, double &dLat, double &dLon) const;
};
