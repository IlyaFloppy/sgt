#include "slam.h"

SLAM::SLAM() {
    detectionMask = cv::imread("mc_mask.png", cv::IMREAD_GRAYSCALE);

    cameraMatrix = cv::Mat1d::zeros(3, 3);
    cameraMatrix.at<double>(0, 0) = 250.77948496;
    cameraMatrix.at<double>(0, 1) = 0;
    cameraMatrix.at<double>(0, 2) = 320;
    cameraMatrix.at<double>(1, 0) = 0;
    cameraMatrix.at<double>(1, 1) = 250.59048828;
    cameraMatrix.at<double>(1, 2) = 200;
    cameraMatrix.at<double>(2, 0) = 0;
    cameraMatrix.at<double>(2, 1) = 0;
    cameraMatrix.at<double>(2, 2) = 1;

    distCoeffs = cv::Mat1d::zeros(5, 1);
    /*distCoeffs.at<double>(0, 0) = 2.85131070e-04;
    distCoeffs.at<double>(1, 0) = -5.09885755e-04;
    distCoeffs.at<double>(2, 0) = 4.23892455e-07;
    distCoeffs.at<double>(3, 0) = -3.94307962e-05;
    distCoeffs.at<double>(4, 0) = 2.13458176e-04;*/

    // detector = cv::FastFeatureDetector::create();
    extractor = cv::ORB::create(
            100,
            1.2,
            3,
            31,
            0,
            2,
            cv::ORB::HARRIS_SCORE,
            31,
            10
    );
    // extractor = cv::BRISK::create();
    detector = extractor;

    matcher = *cv::BFMatcher::create(cv::NORM_L2, false);

    estRotation = Eigen::Quaterniond(1, 0, 0, 0);
    estTranslation = Eigen::Vector3d(0, 0, 0);

//    framesHistory.emplace_back(std::move(FrameSnapshot()));

    const int averageBetweenFrames = 1;
    for (int i = 0; i < averageBetweenFrames; i++) {
        poseHistory.emplace_back(Pose(
                Eigen::Quaterniond(1, 0, 0, 0),
                Eigen::Vector3d(0, 0, 0),
                FrameSnapshot()
        ));
    }
}

SLAM::~SLAM() = default;

// TODO: fill map with projections
void SLAM::feed(const cv::Mat &frame) {
    Eigen::Quaterniond averageRotation(0, 0, 0, 0);
    Eigen::Vector3d averageTranslation(0, 0, 0);
    FrameSnapshot snapshot;
    for (auto &pose:poseHistory) {
        auto poseChange = estimatePoseChange(pose.snapshot, frame);
        auto rotation = poseChange.rotation * pose.rotation;
        // TODO: premultiply change by rotation
        auto translation = poseChange.translation + pose.translation;

        averageTranslation += translation;
        averageRotation = Eigen::Quaterniond(
                averageRotation.w() + rotation.w(),
                averageRotation.x() + rotation.x(),
                averageRotation.y() + rotation.y(),
                averageRotation.z() + rotation.z()
        );

        snapshot = poseChange.snapshot; // use last snapshot
    }
    averageTranslation /= poseHistory.size();
    averageRotation = Eigen::Quaterniond(
            averageRotation.w() / poseHistory.size(),
            averageRotation.x() / poseHistory.size(),
            averageRotation.y() / poseHistory.size(),
            averageRotation.z() / poseHistory.size()
    );

    // do we actually need this?
    // auto translationChange = averageTranslation - poseHistory.back().translation;
    // auto rotationChange = averageRotation * poseHistory.back().rotation.inverse();

    auto pose = Pose(averageRotation, averageTranslation, snapshot);
    poseHistory.emplace_back(pose);
    poseHistory.pop_front();

    path.emplace_back(pose.translation);

    cv::Mat matchesImage;
    cv::drawMatches(
            frame,
            pose.snapshot.keypoints,
            frame,
            pose.snapshot.rKeypoints,
            pose.snapshot.matches,
            matchesImage,
            cv::Scalar::all(-1),
            cv::Scalar::all(-1),
            std::vector<char>(),
            cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS
    );
    imshow("matches", matchesImage);
}

void SLAM::savePathAsImage(const std::string &name, bool volumetric) {
    namespace plt = matplotlibcpp;

    std::vector<double> x, y, z;
    for (auto &point : path) {
        x.emplace_back(point.x());
        y.emplace_back(point.y());
        z.emplace_back(point.z());
    }

    if (volumetric) {
        plt::plot3(x, y, z);
    } else {
        plt::plot(x, y);
        plt::axis("equal");
    }
    plt::grid(true);
    plt::save(name);
    plt::show(true);
}

std::tuple<SLAM::FrameSnapshot, bool> SLAM::makeSnapshot(const cv::Mat &_frame, const FrameSnapshot &relativity) {
    cv::Mat frame;
    cv::undistort(_frame, frame, cameraMatrix, distCoeffs);

    std::vector<cv::KeyPoint> keypoints;
    cv::Mat features;

    detector->detect(frame, keypoints, detectionMask);
    extractor->compute(frame, keypoints, features);

    if (relativity.isCutOff()) {
        return {FrameSnapshot(keypoints, {}, {}, {}, {}, features), true};
    }

    if (keypoints.size() < 50) {
        auto msg = "not enough keypoints: " + std::to_string(keypoints.size());
        return {FrameSnapshot(keypoints, {}, {}, {}, {}, features), true};
    }

    std::vector<cv::DMatch> matches;
    if (relativity.features.size[0] > 20 && features.size[0] > 20) {
        matcher.match(features, relativity.features, matches);
    } else {
        std::cout << "not enough features detected: " << features.size() << std::endl;
        return {FrameSnapshot(keypoints, {}, {}, {}, {}, features), true};
    }

    double maxDistance = 0;
    double minDistance = 100;

    for (int i = 0; i < features.rows; i++) {
        double distance = matches[i].distance;
        if (distance < minDistance) {
            minDistance = distance;
        }
        if (distance > maxDistance) {
            maxDistance = distance;
        }
    }

    std::vector<cv::DMatch> filteredMatches;
    double matchDistance = minDistance < 10 ? 90 : 9 * minDistance;
    auto maxDelta = (frame.cols + frame.rows) / 300;
    for (int i = 0; i < features.rows; i++) {
        if (matches[i].distance <= matchDistance) {
            auto delta = keypoints[matches[i].queryIdx].pt - relativity.keypoints[matches[i].trainIdx].pt;
            if (abs(delta.x) + abs(delta.y) < maxDelta) {
                filteredMatches.emplace_back(matches[i]);
            }
        }
    }

    if (filteredMatches.size() < 10) {
        auto msg = ("not enough matches: " +
                    std::to_string(matches.size()) +
                    " -> " +
                    std::to_string(filteredMatches.size()));
        std::cout << msg << std::endl;
        // TODO: something
        // return {relativity, true};
        return {FrameSnapshot(keypoints, {}, {}, {}, {}, features), true};
    }

    std::vector<cv::Point2f> curPoints(filteredMatches.size());
    std::vector<cv::Point2f> prevPoints(filteredMatches.size());
    int idx = 0;
    for (auto &match : filteredMatches) {
        curPoints[idx] = keypoints[match.queryIdx].pt;
        prevPoints[idx] = relativity.keypoints[match.trainIdx].pt;
        ++idx;
    }

    return {
            SLAM::FrameSnapshot(
                    keypoints,
                    curPoints,
                    relativity.keypoints,
                    prevPoints,
                    filteredMatches,
                    features
            ), false};
}

SLAM::Pose SLAM::estimatePoseChange(const SLAM::FrameSnapshot &from, const cv::Mat &frame) {
    auto[snapshot, cutoff] = makeSnapshot(frame, from);
    if (cutoff) {
        return Pose(
                Eigen::Quaterniond(1, 0, 0, 0),
                Eigen::Vector3d(0, 0, 0),
                snapshot
        );
    }

    cv::Mat essential, rotation, translation, mask;
    essential = findEssentialMat(snapshot.points, snapshot.rPoints, cameraMatrix, cv::RANSAC, 0.999, 0.15, mask);
    recoverPose(essential, snapshot.points, snapshot.rPoints, cameraMatrix, rotation, translation, 99, mask);

    // 1
//    estTranslation += estRotation.toRotationMatrix() * Eigen::Vector3d(
    // 2
    auto translationDifference = Eigen::Vector3d(
            translation.at<double>(0, 0),
            translation.at<double>(1, 0),
            translation.at<double>(2, 0)
    );


    Eigen::Matrix3d frameRotation;
    frameRotation << rotation.at<double>(0, 0),
            rotation.at<double>(0, 1),
            rotation.at<double>(0, 2),
            rotation.at<double>(1, 0),
            rotation.at<double>(1, 1),
            rotation.at<double>(1, 2),
            rotation.at<double>(2, 0),
            rotation.at<double>(2, 1),
            rotation.at<double>(2, 2);


    auto rotationDerivative = Eigen::Quaterniond(frameRotation);
    if (abs(rotationDerivative.w()) < 0.0015)
        rotationDerivative = Eigen::Quaterniond(1, 0, 0, 0);


    return SLAM::Pose(rotationDerivative, translationDifference, snapshot);
}