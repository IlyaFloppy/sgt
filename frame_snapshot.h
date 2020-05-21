#pragma once

#include <vector>

#include <opencv2/opencv.hpp>

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
    }

    bool isCutOff() const {
        return keypoints.empty();
    }
};