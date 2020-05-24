//
// Created by Ilya Popovnin on 2020-05-20.
//

#include "mapper.h"

Mapper::Mapper() {
}

Mapper::~Mapper() {
    for (const auto&[tile, necessity]:tiles) {
        tile.write();
    }
}

void Mapper::feed(const Pose &pose, const cv::Mat &frame) {

    // TODO: feed not only into center tile
    auto coords = Tile::GeoCoords(pose.geo.x(), pose.geo.y());
    Tile tile = getTileAt(coords);
    tile.feed(pose, frame);
    tiles[tile] = 20;

    auto it = tiles.begin();
    while (it != tiles.end()) {
        if (it->second == 0) {
            it->first.write();
            it = tiles.erase(it);
        } else {
            it++->second -= 1;
        }
    }
}

Tile Mapper::getTileAt(const Tile::GeoCoords &coords) {
    for (auto const &t : tiles) {
        if (t.first.contains(coords)) {
            return t.first;
        }
    }

    int latInternal = (int) (coords.lat * Tile::realToInternal / tileSize) * tileSize;
    int lonInternal = (int) (coords.lon * Tile::realToInternal / tileSize) * tileSize;

    Tile tile(latInternal, lonInternal, tileSize);
    return std::move(tile);
}

