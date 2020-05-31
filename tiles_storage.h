#pragma once

#include "tile.h"

class TilesStorage {
public:
    Tile *getTileAt(double lat, double lon);

    Tile *getTileAt(int64_t lat, int64_t lon);

    void disposeUnused(bool all = false);

private:
    std::unordered_map<int64_t, std::pair<Tile *, int>> tiles;
};