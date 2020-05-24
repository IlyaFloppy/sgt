#pragma once

#include <cmath>
#include <filesystem>

#include <opencv2/opencv.hpp>

#include <gdal.h>
#include <gdal_priv.h>

#include "helper.h"
#include "pose.h"

struct Tile {
public:
    struct GeoCoords {
        double lat, lon;

        GeoCoords(double lat, double lon) : lat(lat), lon(lon) {}
    };

    // stored in seconds * 100 internally
    static const int realToInternal = 60 * 60 * 100;
    const int latMin;
    const int latMax;
    const int lonMin;
    const int lonMax;
    const int size;

    const int nRows = 128;
    const int nCols = 128;

    uint8_t *data;

    Tile(int lat,
         int lon,
         int size
    );

    Tile(const Tile &tile);

    Tile(Tile &&tile) noexcept;

    ~Tile();

    void write() const;

    void feed(const Pose &pose, const cv::Mat &frame);

    bool contains(GeoCoords coords) const;

    friend bool operator<(const Tile &lhs, const Tile &rhs);

    friend bool operator==(const Tile &lhs, const Tile &rhs);
};