//
// Created by Ilya Popovnin on 2020-05-26.
//

#include "tiles_storage.h"

Tile *TilesStorage::getTileAt(double lat, double lon) {
    return getTileAt(
            (int64_t) (lat / Tile::size + 0.5) * Tile::size,
            (int64_t) (lon / Tile::size + 0.5) * Tile::size
    );
}

Tile *TilesStorage::getTileAt(int64_t lat, int64_t lon) {
    int64_t id = (lat << 32) + lon;
    auto it = tiles.find(id);
    if (it == tiles.end()) {
        auto tile = new Tile(
                (int) (lat / Tile::size + 0.5) * Tile::size,
                (int) (lon / Tile::size + 0.5) * Tile::size
        );
        tiles[id] = std::make_pair(tile, 20);
        return tile;
    } else {
        return it->second.first;
    }
}

void TilesStorage::disposeUnused(bool all) {
    auto it = tiles.begin();
    while (it != tiles.end()) {
        if (it->second.second == 0 || all) {
            it->second.first->write();
            delete it->second.first;
            it = tiles.erase(it);
        } else {
            it++->second.second -= 1;
        }
    }
}
