//
// Created by Ilya Popovnin on 2020-05-20.
//

#include "mapper.h"

Mapper::Mapper() {
    GDALDriverManager *pDriverManager = GetGDALDriverManager();
    pDriver = pDriverManager->GetDriverByName("GTiff");
}

Mapper::~Mapper() {
    GDALDestroyDriverManager();
}

void Mapper::feed(const Pose &pose, const cv::Mat &frame) {
    // TODO: save frames into tiles
}

Tile const &Mapper::getTileAt(double latitude, double longitude) {
    for (auto const &t : tiles) {
        if (t.first.contains(latitude, longitude)) {
            return t.first;
        }
    }

    int size = 10; // internal units
    int latInternal = (int)(latitude * Tile::realToInternal / size) * size;
    int lonInternal = (int)(longitude * Tile::realToInternal / size) * size;

    Tile tile(latInternal, lonInternal, size, pDriver);
    tiles[tile] = 10;
    return getTileAt(latitude, longitude);
}

