#pragma once

#include <utility>
#include <vector>
#include <deque>
#include <tuple>
#include <string>
#include <cmath>

#include <opencv2/opencv.hpp>

#include "Eigen/Geometry"

#include "matplotlibcpp.h"

class SLAM {
public:
    SLAM();

    ~SLAM();

    void feed(const cv::Mat &frame);

    void savePathAsImage(const std::string &name, bool volumetric);

    struct FrameSnapshot {
        // used for matching
        std::vector<cv::KeyPoint> keypoints;
        std::vector<cv::KeyPoint> rKeypoints;
        // used for restoring pose
        std::vector<cv::Point2f> points;
        std::vector<cv::Point2f> rPoints;

        std::vector<cv::DMatch> matches;

        cv::Mat features;

        FrameSnapshot() {};

        FrameSnapshot(const FrameSnapshot &snapshot) {
            keypoints = snapshot.keypoints;
            rKeypoints = snapshot.rKeypoints;
            points = snapshot.points;
            rPoints = snapshot.rPoints;
            matches = snapshot.matches;
            features = snapshot.features;
        }

        FrameSnapshot(
                std::vector<cv::KeyPoint> keypoints,
                std::vector<cv::Point2f> points,
                std::vector<cv::KeyPoint> rKeypoints,
                std::vector<cv::Point2f> rPoints,
                std::vector<cv::DMatch> matches,
                cv::Mat features)
                : keypoints(std::move(keypoints)),
                  rKeypoints(std::move(rKeypoints)),
                  points(std::move(points)),
                  rPoints(std::move(rPoints)),
                  matches(std::move(matches)),
                  features(std::move(features)) {
//            std::cout << this->matches.size() << std::endl;
        }

        bool isCutOff() const {
            return keypoints.empty();
        }
    };

    std::tuple<FrameSnapshot, bool> makeSnapshot(const cv::Mat &_frame, const FrameSnapshot &relativity);

    struct PoseSnapshot {
        PoseSnapshot(Eigen::Quaterniond rotation,
                     Eigen::Vector3d translation)
                : rotation(std::move(rotation)),
                  translation(std::move(translation)) {};
        // relative to one of previous frames
        Eigen::Quaterniond rotation;
        Eigen::Vector3d translation;
    };

private:
    cv::Ptr<cv::FeatureDetector> detector;
    cv::Ptr<cv::DescriptorExtractor> extractor;

    cv::Mat detectionMask;

    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;

//    std::vector<cv::KeyPoint> previousKeypoints;
//    cv::Mat previousFeatures;
    cv::BFMatcher matcher;

    std::deque<FrameSnapshot> framesHistory;

    Eigen::Quaterniond estRotation;
    Eigen::Vector3d estTranslation;
    std::vector<Eigen::Vector3d> path;
};