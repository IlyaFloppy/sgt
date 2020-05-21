#pragma once

#include "frame_snapshot.h"

#include "Eigen/Geometry"

struct Pose {
    // relative to one of previous frames when used as change, absolute values otherwise
    Eigen::Quaterniond rotation;
    Eigen::Vector3d translation;
    Eigen::Vector2d geo;
    FrameSnapshot snapshot;

    Pose(Eigen::Quaterniond rotation,
         Eigen::Vector3d translation,
         Eigen::Vector2d geo,
         const FrameSnapshot &snapshot)
            : rotation(std::move(rotation)),
              translation(std::move(translation)),
              geo(std::move(geo)),
              snapshot(snapshot) {};
};