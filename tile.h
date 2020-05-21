#pragma once

#include <gdal.h>
#include <gdal_priv.h>

struct Tile {
    // stored in seconds * 100 internally
    static const int realToInternal = 60 * 60 * 100;
    const int latMin;
    const int latMax;
    const int lonMin;
    const int lonMax;

    GDALDataset *pDataset;

    const int nRows = 1024;
    const int nCols = 1024;

    Tile(int lat,
         int lon,
         int size,
         GDALDriver *pDriver
    );

    ~Tile();

    bool contains(double lat, double lon) const;

    friend bool operator <(const Tile& lhs, const Tile& rhs);
};