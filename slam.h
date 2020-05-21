#pragma once

#include <utility>
#include <vector>
#include <deque>
#include <tuple>
#include <string>
#include <cmath>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#include <gdal.h>
#include <gdal_priv.h>

#include "Eigen/Geometry"

#include "matplotlibcpp.h"
#include "frame_snapshot.h"
#include "pose.h"
#include "mapper.h"

class SLAM {
public:
    SLAM();

    ~SLAM();

    void feed(const cv::Mat &frame);

    void savePathAsImage(const std::string &name, bool volumetric);

    std::tuple<FrameSnapshot, bool> makeSnapshot(const cv::Mat &frame, const FrameSnapshot &relativity);

    // sounds good, doesn't work
    Pose estimatePoseChange3D(const FrameSnapshot &from, const cv::Mat &frame);

    // this should work. sometimes.
    Pose estimatePoseChange2D(const Pose &from, const cv::Mat &frame);

private:
    double pixelSize;
    double focalLength;
    double frameWidthPx;
    double frameHeightPx;

    cv::Ptr<cv::FeatureDetector> detector;
    cv::Ptr<cv::DescriptorExtractor> extractor;

    cv::Mat detectionMask;

    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;

    cv::BFMatcher matcher;

    std::deque<Pose> poseHistory;
    std::vector<Eigen::Vector3d> path;
    std::vector<Eigen::Vector2d> geoPath;

    Mapper mapper;
};