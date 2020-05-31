#pragma once

#include "frame_snapshot.h"
#include "helper.h"

#include "Eigen/Geometry"

struct Pose {
    struct FrameInfo {
        double frameWidthPx;
        double frameHeightPx;
        double frameWidthMeters;
        double frameHeightMeters;

        FrameInfo(double frameWidthPx, double frameHeightPx, double frameWidthMeters, double frameHeightMeters);
    };

    // relative to one of previous frames when used as change, absolute values otherwise
    Eigen::Quaterniond rotation;
    Eigen::Vector3d translation;
    Eigen::Vector2d geo;
    FrameSnapshot snapshot;
    FrameInfo frameInfo;

    Pose(Eigen::Quaterniond rotation,
         Eigen::Vector3d translation,
         Eigen::Vector2d geo,
         const FrameSnapshot &snapshot,
         FrameInfo frameInfo);

    double geoSize() const;

    bool isPointInFrame(double x, double y) const;
};