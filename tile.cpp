//
// Created by Ilya Popovnin on 2020-05-21.
//

#include "tile.h"

Tile::Tile(int lat, int lon, int size) :
        latMin(lat - size / 2),
        latMax(lat + size / 2),
        lonMin(lon - size / 2),
        lonMax(lon + size / 2),
        size(size) {
    data = new uint8_t[nRows * nCols];
    std::string filename =
            "tiles/" +
            std::to_string(latMin) + ", " +
            std::to_string(lonMin) + ", " +
            std::to_string(size) + ".tiff";

    GDALAllRegister();
    GDALDriverManager *pDriverManager = GetGDALDriverManager();
    GDALDriver *pDriver = pDriverManager->GetDriverByName("GTiff");
    GDALDataset *pDataset;
    if (std::filesystem::exists(filename)) {
        pDataset = (GDALDataset *) GDALOpen(filename.c_str(), GA_Update);

        CPLErr ignored = pDataset->GetRasterBand(1)->RasterIO(
                GF_Read, 0, 0, nCols, nRows, data, nCols, nRows, GDT_Byte, 0, 0);

        GDALClose(pDataset);
    } else {
        for (int i = 0; i < nRows * nCols; ++i) {
            data[i] = 255;
        }
    }
}

Tile::Tile(const Tile &tile)
        : latMin(tile.latMin),
          latMax(tile.latMax),
          lonMin(tile.lonMin),
          lonMax(tile.lonMax),
          size(tile.size) {
    data = new uint8_t[nRows * nCols];
    for (int i = 0; i < nRows * nCols; ++i) {
        data[i] = tile.data[i];
    }
}

Tile::Tile(Tile &&tile) noexcept
        : latMin(tile.latMin),
          latMax(tile.latMax),
          lonMin(tile.lonMin),
          lonMax(tile.lonMax),
          size(tile.size) {
    data = tile.data;
    tile.data = nullptr;
}

Tile::~Tile() {
    delete[] data;
}

void Tile::write() const {

    std::string filename =
            "tiles/" +
            std::to_string(latMin) + ", " +
            std::to_string(lonMin) + ", " +
            std::to_string(size) + ".tiff";

    GDALAllRegister();
    GDALDriverManager *pDriverManager = GetGDALDriverManager();
    GDALDriver *pDriver = pDriverManager->GetDriverByName("GTiff");
    GDALDataset *pDataset;
    if (std::filesystem::exists(filename)) {
        pDataset = (GDALDataset *) GDALOpen(filename.c_str(), GA_Update);
    } else {
        pDataset = pDriver->Create(
                filename.c_str(),
                nCols,
                nRows,
                1,
                GDALDataType::GDT_Byte,
                nullptr
        );
    }

    CPLErr ignored = pDataset->GetRasterBand(1)->RasterIO(
            GF_Write, 0, 0, nCols, nRows, data, nCols, nRows, GDT_Byte, 0, 0);

    double xMin = fmin(lonMin, lonMax) / realToInternal;
    double xMax = fmax(lonMin, lonMax) / realToInternal;
    double yMin = fmin(latMin, latMax) / realToInternal;
    double yMax = fmax(latMin, latMax) / realToInternal;

    double xRes = (xMax - xMin) / nCols;
    double yRes = (yMax - yMin) / nRows;

    double trans[6] = {xMin, xRes, 0, yMax, 0, -yRes};
//    double trans[6] = {yMin, yRes, 0, xMax, 0, -xRes};
    pDataset->SetGeoTransform(trans);
    GDALClose(pDataset);
    // TODO: causes segfault in open/create. wtf
    // GDALDestroyDriver(pDriver);
    // GDALDestroyDriverManager();
}

bool Tile::contains(GeoCoords coords) const {
    double latInternal = coords.lat * realToInternal;
    double lonInternal = coords.lon * realToInternal;
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

bool operator==(const Tile &lhs, const Tile &rhs) {
    return lhs.latMin == rhs.latMin &&
           lhs.latMax == rhs.latMax &&
           lhs.lonMin == rhs.lonMin &&
           lhs.lonMax == rhs.lonMax;
}

void Tile::feed(const Pose &pose, const cv::Mat &frame) {
    Tile::GeoCoords coords(0, 0);

    for (int y = 0; y < nRows; ++y) {
        coords.lat = (latMin + (double) y * size / nRows) / realToInternal;
        for (int x = 0; x < nCols; ++x) {
            coords.lon = (lonMin + (double) x * size / nCols) / realToInternal;

            auto deltaGeoMeters = Eigen::Vector2d(
                    (coords.lat - pose.geo.x()) / 2 / M_PI * helper::WGS84_SMALL_RAD,
                    (coords.lon - pose.geo.y()) / 2 / M_PI * helper::WGS84_BIG_RAD / cos(coords.lat / 180 * M_PI)
            );

            auto deltaFrameMeters = pose.rotation.toRotationMatrix()
                                            .block(0, 0, 2, 2) * deltaGeoMeters;

            auto pixelLocation = Eigen::Vector2i(
                    (deltaFrameMeters.x() / pose.frameInfo.frameWidthMeters + 0.5) * pose.frameInfo.frameWidthPx,
                    (deltaFrameMeters.y() / pose.frameInfo.frameHeightMeters + 0.5) * pose.frameInfo.frameHeightPx
            );

            if (pixelLocation.x() < 10 ||
                pixelLocation.x() >= pose.frameInfo.frameWidthPx - 10 ||
                pixelLocation.y() < 10 ||
                pixelLocation.y() >= pose.frameInfo.frameHeightPx - 10) {
                continue;
            } else {
                // TODO: antialias
                uint8_t pixel = frame.at<uint8_t>(pixelLocation.x(), pixelLocation.y());
                data[x + y * nCols] = pixel;
            }
        }
    }
}