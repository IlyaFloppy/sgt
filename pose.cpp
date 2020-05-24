//
// Created by Ilya Popovnin on 2020-05-21.
//

#include "pose.h"

Pose::FrameInfo::FrameInfo(
        double frameWidthPx,
        double frameHeightPx,
        double frameWidthMeters,
        double frameHeightMeters)
        : frameWidthPx(frameWidthPx),
          frameHeightPx(frameHeightPx),
          frameWidthMeters(frameWidthMeters),
          frameHeightMeters(frameHeightMeters) {}

Pose::Pose(
        Eigen::Quaterniond rotation,
        Eigen::Vector3d translation,
        Eigen::Vector2d geo,
        const FrameSnapshot &snapshot,
        Pose::FrameInfo frameInfo)
        : rotation(std::move(rotation)),
          translation(std::move(translation)),
          geo(std::move(geo)),
          snapshot(snapshot),
          frameInfo(std::move(frameInfo)) {
}

double Pose::geoSize() const {
    return fmax(frameInfo.frameWidthMeters, frameInfo.frameHeightMeters) / helper::WGS84_SMALL_LENGTH * 360;
}
