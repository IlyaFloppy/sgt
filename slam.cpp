#include "slam.h"
#include "helper.h"

SLAM::SLAM() {
//    detectionMask = cv::imread("mc_mask.png", cv::IMREAD_GRAYSCALE);

    pixelSize = 1.4 * 1e-6; // meters
    focalLength = 1.8 * 1e-3; // meters

    frameWidthPx = 640;
    frameHeightPx = 480;

    pixelSize *= 10;

    cameraMatrix = cv::Mat1d::zeros(3, 3);
    distCoeffs = cv::Mat1d::zeros(5, 1);

    // minecraft at 640x400
    cameraMatrix.at<double>(0, 0) = 250; //250.77948496;
    cameraMatrix.at<double>(0, 1) = 0;
    cameraMatrix.at<double>(0, 2) = 320;
    cameraMatrix.at<double>(1, 0) = 0;
    cameraMatrix.at<double>(1, 1) = 250; //250.59048828;
    cameraMatrix.at<double>(1, 2) = 200;
    cameraMatrix.at<double>(2, 0) = 0;
    cameraMatrix.at<double>(2, 1) = 0;
    cameraMatrix.at<double>(2, 2) = 1;
    /*distCoeffs.at<double>(0, 0) = 2.85131070e-04;
    distCoeffs.at<double>(1, 0) = -5.09885755e-04;
    distCoeffs.at<double>(2, 0) = 4.23892455e-07;
    distCoeffs.at<double>(3, 0) = -3.94307962e-05;
    distCoeffs.at<double>(4, 0) = 2.13458176e-04;*/

    // iphone xr camera at 640x480
    cameraMatrix.at<double>(0, 0) = 487.8492598; //250.77948496;
    cameraMatrix.at<double>(0, 1) = 0;
    cameraMatrix.at<double>(0, 2) = 315.759412;
    cameraMatrix.at<double>(1, 0) = 0;
    cameraMatrix.at<double>(1, 1) = 487.8363496; //250.59048828;
    cameraMatrix.at<double>(1, 2) = 227.98274297;
    cameraMatrix.at<double>(2, 0) = 0;
    cameraMatrix.at<double>(2, 1) = 0;
    cameraMatrix.at<double>(2, 2) = 1;

    distCoeffs.at<double>(0, 0) = 1.93099278e-01;
    distCoeffs.at<double>(1, 0) = -8.82239994e-01;
    distCoeffs.at<double>(2, 0) = 7.81156833e-05;
    distCoeffs.at<double>(3, 0) = 6.71192996e-04;
    distCoeffs.at<double>(4, 0) = 1.18223036e+00;


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
    detector = extractor;

    matcher = *cv::BFMatcher::create(cv::NORM_L2, false);

//    framesHistory.emplace_back(std::move(FrameSnapshot()));

    const int averageBetweenFrames = 6;
    for (int i = 0; i < averageBetweenFrames; i++) {
        poseHistory.emplace_back(Pose(
                Eigen::Quaterniond(1, 0, 0, 0),
                Eigen::Vector3d(0, 0, 500),
                Eigen::Vector2d(55.761495, 37.712632),
                FrameSnapshot(),
                Pose::FrameInfo(frameWidthPx, frameHeightPx, 0, 0)
        ));
    }
}

SLAM::~SLAM() {
    GDALDestroyDriverManager();
};

void SLAM::feed(cv::Mat &frame) {
    Eigen::Quaterniond averageRotation(0, 0, 0, 0);
    Eigen::Vector3d averageTranslation(0, 0, 0);
    Eigen::Vector2d averageGeoTranslation(0, 0);
    FrameSnapshot snapshot;
    double frameWidthMeters = 0, frameHeightMeters = 0;
    for (auto &pose:poseHistory) {
        auto poseChange = estimatePoseChange2D(pose, frame);
        auto rotation = poseChange.rotation * pose.rotation;
        auto translation = pose.rotation.toRotationMatrix() * poseChange.translation + pose.translation;
        auto geoTranslation = pose.rotation.toRotationMatrix()
                                      .block(0, 0, 2, 2) * poseChange.geo + pose.geo;

        averageTranslation += translation;
        averageGeoTranslation += geoTranslation;
        averageRotation = Eigen::Quaterniond(
                averageRotation.w() + rotation.w(),
                averageRotation.x() + rotation.x(),
                averageRotation.y() + rotation.y(),
                averageRotation.z() + rotation.z()
        );

        snapshot = poseChange.snapshot; // use last snapshot

        frameWidthMeters += poseChange.frameInfo.frameWidthMeters;
        frameHeightMeters += poseChange.frameInfo.frameHeightMeters;
    }
    averageTranslation /= poseHistory.size();
    averageGeoTranslation /= poseHistory.size();
    averageRotation = Eigen::Quaterniond(
            averageRotation.w() / poseHistory.size(),
            averageRotation.x() / poseHistory.size(),
            averageRotation.y() / poseHistory.size(),
            averageRotation.z() / poseHistory.size()
    );

    frameWidthMeters /= poseHistory.size();
    frameHeightMeters /= poseHistory.size();

    // do we actually need this?
    // auto translationChange = averageTranslation - poseHistory.back().translation;
    // auto rotationChange = averageRotation * poseHistory.back().rotation.inverse();

    auto pose = Pose(
            averageRotation,
            averageTranslation,
            averageGeoTranslation,
            snapshot,
            Pose::FrameInfo(frameWidthPx, frameHeightPx, frameWidthMeters, frameHeightMeters));
    poseHistory.emplace_back(pose);
    poseHistory.pop_front();

    path.emplace_back(pose.translation);
    geoPath.emplace_back(pose.geo);

    mapper.feed(pose, frame);

    cv::Mat matchesImage;
//    cv::drawMatches(
//            frame,
//            pose.snapshot.keypoints,
//            frame,
//            pose.snapshot.rKeypoints,
//            pose.snapshot.matches,
//            matchesImage,
//            cv::Scalar::all(-1),
//            cv::Scalar::all(-1),
//            std::vector<char>(),
//            cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS
//    );
//    imshow("matches", matchesImage);
//    imshow("frame", frame);
}

void SLAM::savePathAsImage(const std::string &name, bool volumetric) {
    namespace plt = matplotlibcpp;

    std::vector<double> x, y, z;
    for (auto &point : path) {
        x.emplace_back(point.x());
        y.emplace_back(point.y());
        z.emplace_back(point.z());
    }

    std::vector<double> lat, lon;
    for (auto &point : geoPath) {
        lat.emplace_back(point.x());
        lon.emplace_back(point.y());
    }
    std::cout << geoPath.size() << std::endl;

    if (volumetric) {
        plt::plot3(x, y, z);
    } else {
        plt::plot(lon, lat);
        plt::axis("equal");
    }
    plt::grid(true);
    plt::save(name);
    plt::show(true);
}

std::tuple<FrameSnapshot, bool> SLAM::makeSnapshot(const cv::Mat &_frame, const FrameSnapshot &relativity) {
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
    double matchDistance = minDistance < 10 ? 150 : 15 * minDistance;
    auto maxDelta = (frame.cols + frame.rows) / 100;
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
            FrameSnapshot(
                    keypoints,
                    curPoints,
                    relativity.keypoints,
                    prevPoints,
                    filteredMatches,
                    features
            ), false};
}

Pose SLAM::estimatePoseChange3D(const FrameSnapshot &from, const cv::Mat &frame) {
    auto[snapshot, cutoff] = makeSnapshot(frame, from);
    if (cutoff) {
        return Pose(
                Eigen::Quaterniond(1, 0, 0, 0),
                Eigen::Vector3d(0, 0, 0),
                Eigen::Vector2d(0, 0),
                snapshot,
                Pose::FrameInfo(frameWidthPx, frameHeightPx, 0, 0)
        );
    }

    cv::Mat essential, rotation, translation, mask;
    essential = findEssentialMat(snapshot.points, snapshot.rPoints, cameraMatrix, cv::RANSAC, 0.999, 0.15, mask);
    recoverPose(essential, snapshot.points, snapshot.rPoints, cameraMatrix, rotation, translation, 99, mask);

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

    // TODO: estimatePoseChange3D is not used anyway
    Eigen::Vector2d geoTranslation(0, 0);

    return Pose(
            rotationDerivative,
            translationDifference,
            geoTranslation,
            snapshot,
            Pose::FrameInfo(frameWidthPx, frameHeightPx, 0, 0)
    );
}

Pose SLAM::estimatePoseChange2D(const Pose &from, const cv::Mat &frame) {
    auto[snapshot, cutoff] = makeSnapshot(frame, from.snapshot);
    if (cutoff) {
        return Pose(
                Eigen::Quaterniond(1, 0, 0, 0),
                Eigen::Vector3d(0, 0, 0),
                Eigen::Vector2d(0, 0),
                snapshot,
                Pose::FrameInfo(frameWidthPx, frameHeightPx, 0, 0)
        );
    }

    int matches = snapshot.matches.size();
    int lines = matches * (matches - 1) / 2;
    // distances
    Eigen::MatrixXd dPrev(lines, 1);
    Eigen::MatrixXd dCur(lines, 1);
    // angles
    Eigen::MatrixXd aPrev(lines, 1);
    Eigen::MatrixXd aCur(lines, 1);
    // translations
    Eigen::MatrixXd translations(matches, 2);

    /* Eigen::MatrixXd fromKp(matches * 2, 6);
    Eigen::MatrixXd toKp(matches * 2, 1);

    int matchIdx = 0;
    for (auto &match : snapshot.matches) {
        auto currentFramePoint = snapshot.keypoints[match.queryIdx].pt;
        auto prevFramePoint = snapshot.rKeypoints[match.trainIdx].pt;

        fromKp(matchIdx * 2, 0) = prevFramePoint.x;
        fromKp(matchIdx * 2, 1) = prevFramePoint.y;
        fromKp(matchIdx * 2, 2) = 0;
        fromKp(matchIdx * 2, 3) = 0;
        fromKp(matchIdx * 2, 4) = 1;
        fromKp(matchIdx * 2, 5) = 0;
        fromKp(matchIdx * 2 + 1, 0) = 0;
        fromKp(matchIdx * 2 + 1, 1) = 0;
        fromKp(matchIdx * 2 + 1, 2) = prevFramePoint.x;
        fromKp(matchIdx * 2 + 1, 3) = prevFramePoint.y;
        fromKp(matchIdx * 2 + 1, 4) = 0;
        fromKp(matchIdx * 2 + 1, 5) = 1;

        toKp(matchIdx, 0) = currentFramePoint.x;
        toKp(matchIdx + 1, 0) = currentFramePoint.y;
    }

    auto fromKpT = fromKp.transpose();
    auto transform = (fromKpT * fromKp).inverse() * fromKpT * toKp;

    double m1 = transform(0, 0);
    double m2 = transform(1, 0);
    double m3 = transform(2, 0);
    double m4 = transform(3, 0);
    double tx = transform(4, 0);
    double ty = transform(5, 0);

    double angle = 2 * atan2(-m2 * m2 - sqrt(1 - m2 * m2) + 1, m2 * sqrt(1 - m2 * m2));
    double scale = m1 / sqrt(1 - m2 * m2);
    Eigen::Vector2d translation2DPx(tx, ty);

    std::cerr << transform << std::endl; */

    int match1Idx = 0;
    int match2Idx = 0;
    int lineIdx = 0;
    for (auto &match1:snapshot.matches) {
        auto currentFramePoint1 = snapshot.keypoints[match1.queryIdx].pt;
        auto prevFramePoint1 = snapshot.rKeypoints[match1.trainIdx].pt;

        translations(match1Idx, 0) = currentFramePoint1.x - prevFramePoint1.x;
        translations(match1Idx, 1) = currentFramePoint1.y - prevFramePoint1.y;

        match2Idx = 0;

        for (auto &match2:snapshot.matches) {
            if (match1Idx >= match2Idx) {
                ++match2Idx;
                continue;
            }
            ++match2Idx;

            auto currentFramePoint2 = snapshot.keypoints[match2.queryIdx].pt;
            auto dxCur = currentFramePoint1.x - currentFramePoint2.x;
            auto dyCur = currentFramePoint1.y - currentFramePoint2.y;
            dCur(lineIdx, 0) = sqrt(dxCur * dxCur + dyCur * dyCur);
            aCur(lineIdx, 0) = atan2(dyCur, dxCur);


            auto prevFramePoint2 = snapshot.rKeypoints[match2.trainIdx].pt;
            auto dxPrev = prevFramePoint1.x - prevFramePoint2.x;
            auto dyPrev = prevFramePoint1.y - prevFramePoint2.y;
            dPrev(lineIdx, 0) = sqrt(dxPrev * dxPrev + dyPrev * dyPrev);
            aPrev(lineIdx, 0) = atan2(dyPrev, dxPrev);

            ++lineIdx;
        }

        ++match1Idx;
    }
    auto dPrevT = dPrev.transpose();
    auto s = (dPrevT * dPrev).inverse() * dPrevT * dCur; // 1x1 matrix
    double scale = s(0, 0);

    auto aChange = aCur - aPrev;
    double angle = aChange.mean();

    auto translation2DPx = translations.colwise().mean();
    double distanceToBottom = from.translation.z();
    double frameWidthMeters = frameWidthPx * pixelSize * (distanceToBottom / focalLength - 1);
    double frameHeightMeters = frameHeightPx * pixelSize * (distanceToBottom / focalLength - 1);
    auto translation2DMeters = Eigen::Vector2d(
            translation2DPx.x() / frameWidthPx * frameWidthMeters,
            translation2DPx.y() / frameHeightPx * frameHeightMeters
    );

    auto rotatedTranslation =
            from.rotation.toRotationMatrix().block(0, 0, 2, 2) * translation2DMeters;

    // assume translation is small
    // x -> north, y -> east
    double latitude = from.geo.x();
    double longitude = from.geo.y();
    double latitudeChange = rotatedTranslation.y() / helper::WGS84_SMALL_LENGTH * 180;
    double longitudeChange = -rotatedTranslation.x() / helper::WGS84_BIG_LENGTH * 180 * cos(latitude / 180 * M_PI);
    Eigen::Vector2d geoTranslation(latitudeChange, longitudeChange);

    return Pose(
            Eigen::Quaterniond(
                    cos(angle / 2),
                    0,
                    0,
                    sin(angle / 2)
            ),
            Eigen::Vector3d(
                    -translation2DMeters.x(),
                    translation2DMeters.y(),
                    from.translation.z() * (1 - scale)
            ),
            geoTranslation,
            snapshot,
            Pose::FrameInfo(frameWidthPx, frameHeightPx, frameWidthMeters, frameHeightMeters)
    );
}