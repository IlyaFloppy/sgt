#pragma once

#include "pose.h"
#include "tile.h"

#include <gdal.h>
#include <gdal_priv.h>

#include <opencv2/opencv.hpp>

class Mapper {
public:
    Mapper();

    ~Mapper();

    void feed(const Pose &pose, const cv::Mat &frame);

private:
    GDALDriver *pDriver;

    // Tile -> priority
    // priority decreases by one on each feed if the tile is not used
    // once it reaches zero Tile is removed from tiles
    std::map<Tile, int> tiles;

    Tile const &getTileAt(double latitude, double longitude);
};
