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
    static const int size = 2000;

    const int nRows = 256 * 2;
    const int nCols = 256 * 2;

    bool finalized = false;

    uint8_t *data;

    Tile(int lat, int lon);

    Tile(const Tile &tile);

    Tile(Tile &&tile) noexcept;

    ~Tile();

    void write() const;

    void set(double latInternal, double lonInternal, uint8_t value);

    bool contains(GeoCoords coords) const;

    bool contains(double latInternal, double lonInternal) const;

    friend bool operator<(const Tile &lhs, const Tile &rhs);

    friend bool operator==(const Tile &lhs, const Tile &rhs);
};