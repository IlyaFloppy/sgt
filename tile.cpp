//
// Created by Ilya Popovnin on 2020-05-21.
//

#include "tile.h"

Tile::Tile(int lat, int lon, int size, GDALDriver *pDriver) :
        latMin(lat - size / 2),
        latMax(lat + size / 2),
        lonMin(lon - size / 2),
        lonMax(lon + size / 2) {
    std::string filename =
            "tiles/" +
            std::to_string(latMin) + ", " +
            std::to_string(lonMin) + ", " +
            std::to_string(size) + ".tiff";
    pDataset = (GDALDataset *) GDALOpen(filename.c_str(), GA_Update);
    if (pDataset == nullptr) {
        pDataset = pDriver->Create(
                filename.c_str(),
                nCols,
                nRows,
                1,
                GDALDataType::GDT_Byte,
                nullptr
        );
    }
}

Tile::~Tile() {
    double xMin = fmin(lonMin, lonMax) / realToInternal;
    double xMax = fmax(lonMin, lonMax) / realToInternal;
    double yMin = fmin(latMin, latMax) / realToInternal;
    double yMax = fmax(latMin, latMax) / realToInternal;

    double xRes = (xMax - xMin) / nCols;
    double yRes = (yMax - yMin) / nRows;

    double trans[6] = {xMin, xRes, 0, yMax, 0, -yRes};
    pDataset->SetGeoTransform(trans);
    GDALClose(pDataset);
}

inline bool Tile::contains(double lat, double lon) const {
    double latInternal = lat * realToInternal;
    double lonInternal = lon * realToInternal;
    return (latInternal >= latMin &&
            latInternal < latMax &&
            lonInternal >= lonMin &&
            lonInternal < lonMax);
}

bool operator<(const Tile &lhs, const Tile &rhs) {
    return lhs.latMin < rhs.latMin ||
           lhs.latMax < rhs.latMax ||
           lhs.lonMin < rhs.lonMin ||
           lhs.lonMax < rhs.lonMax;
}
