//
// Created by Ilya Popovnin on 2020-05-20.
//

#include "mapper.h"

Mapper::Mapper() {
}

Mapper::~Mapper() {
    tiles.disposeUnused(true);
}

void Mapper::feed(const Pose &pose, cv::Mat &frame) {

    int width = pose.frameInfo.frameWidthPx;
    int height = pose.frameInfo.frameHeightPx;
    double lat, lon;

    Eigen::MatrixXd A(4, 3);
    Eigen::MatrixXd b(4, 2);

    int idx = 0;
    for (int i : {-1, 1}) {
        for (int j : {-1, 1}) {
            A(idx, 0) = width / 2 * i;
            A(idx, 1) = height / 2 * j;
            A(idx, 2) = 1;
            getInternalCoords(pose, width / 2 * i, height / 2 * j, lat, lon);
            b(idx, 0) = -lat;
            b(idx, 1) = lon;
            ++idx;
        }
    }

    auto transform = (A.transpose() * A).inverse() * A.transpose() * b;
    double rLat1 = transform(0, 0);
    double rLat2 = transform(1, 0);
    double tLat = transform(2, 0) + pose.geo.x() * Tile::realToInternal;
    double rLon1 = transform(0, 1);
    double rLon2 = transform(1, 1);
    double tLon = transform(2, 1) + pose.geo.y() * Tile::realToInternal;

    int fx, fy;
    for (int x = -width / 2; x < width / 2; x += 1) {
        for (int y = -height / 2; y < height / 2; y += 1) {
            // getInternalCoords is almost linear; interpolate between corners
            // getInternalCoords(pose, x, y, lat, lon);
            // lat = pose.geo.x() * Tile::realToInternal - lat;
            // lon = pose.geo.y() * Tile::realToInternal + lon;
            lat = rLat1 * x + rLat2 * y + tLat;
            lon = rLon1 * x + rLon2 * y + tLon;
            fx = x + width / 2;
            fy = y + height / 2;
            Tile *tile = tiles.getTileAt(lat, lon);

            /* if (abs(lat - tile->latMin) < 15) { frame.at<uint8_t>(fy, fx) = 255; }
            if (abs(lat - tile->latMax) < 15) { frame.at<uint8_t>(fy, fx) = 255; }
            if (abs(lon - tile->lonMin) < 15) { frame.at<uint8_t>(fy, fx) = 255; }
            if (abs(lon - tile->lonMax) < 15) { frame.at<uint8_t>(fy, fx) = 255; } */

            if (tile->contains(lat, lon)) {
                tile->set(lat, lon, frame.at<uint8_t>(fy, fx));
            }
        }
    }
    tiles.disposeUnused();
}


void Mapper::getInternalCoords(const Pose &pose, double dy, double dx, double &dLat, double &dLon) const {
    double alpha = -pose.rotation.angularDistance(Eigen::Quaterniond(1, 0, 0, 0));

    dx *= pose.frameInfo.frameWidthMeters / pose.frameInfo.frameWidthPx;
    dy *= pose.frameInfo.frameHeightMeters / pose.frameInfo.frameHeightPx;

    double dLatMeters = dx * cos(alpha) - dy * sin(alpha);
    double dLonMeters = dy * cos(alpha) + dx * sin(alpha);

    dLat = (dLatMeters / helper::WGS84_SMALL_LENGTH * 360) * Tile::realToInternal;
    dLon = (dLonMeters / helper::WGS84_BIG_LENGTH * 360 * cos(pose.geo.x() / 180 * M_PI)) * Tile::realToInternal;
}