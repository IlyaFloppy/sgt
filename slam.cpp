#include "slam.h"

SLAM::SLAM() {
    detectionMask = cv::imread("mc_mask.png", cv::IMREAD_GRAYSCALE);

    pixelSize = 1.4 * 1e-6; // meters
    focalLength = 1.8 * 1e-3; // meters
    frameWidthPx = 640;
    frameHeightPx = 400;

    pixelSize *= 100;

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
            300,
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

    const int averageBetweenFrames = 3;
    for (int i = 0; i < averageBetweenFrames; i++) {
        poseHistory.emplace_back(Pose(
                Eigen::Quaterniond(1, 0, 0, 0),
                Eigen::Vector3d(0, 0, 500),
                Eigen::Vector2d(55.761495, 37.712632),
                FrameSnapshot()
        ));
    }

//    GDALAllRegister();
//    const int nRows = 512;
//    const int nCols = 512;
//    GDALDriverManager *pDriverManager = GetGDALDriverManager();
//    GDALDriver *pDriver = pDriverManager->GetDriverByName("GTiff");
//    GDALDataset *pDataset = pDriver->Create("out.tiff", nCols, nRows, 1, GDALDataType::GDT_Byte, nullptr);
//
//    for (int i = 0; i < nRows; ++i) {
////        auto *testRow = (uint8_t *) CPLMalloc(sizeof(uint8_t) * nCols);
//        auto *testRow = new uint8_t[sizeof(uint8_t) * nCols];
//        for (int j = 0; j < nCols; ++j) {
//            testRow[j] = (i + j * i / 300 + j) % 256;
//        }
//        CPLErr ignored = pDataset->GetRasterBand(1)->RasterIO(
//                GF_Write, 0, i, nCols, 1, testRow, nCols, 1, GDT_Byte, 0, 0);
//        delete[] testRow;
//    }
//
//    std::pair<double, double> latitude = {39, 38.5};
//    std::pair<double, double> longitude = {-120, -119.5};
//    double xMin = fmin(longitude.first, longitude.second);
//    double yMin = fmin(latitude.first, latitude.second);
//    double xMax = fmax(longitude.first, longitude.second);
//    double yMax = fmax(latitude.first, latitude.second);
//
//    double xRes = (xMax - xMin) / nCols;
//    double yRes = (yMax - yMin) / nRows;
//
//    double trans[6] = {xMin, xRes, 0, yMax, 0, -yRes};
//    pDataset->SetGeoTransform(trans);
////    auto err = pDataset->SetProjection("WGS_84");
//    GDALClose(pDataset);
}

SLAM::~SLAM() {
    GDALDestroyDriverManager();
};

// TODO: fill map with projections
void SLAM::feed(const cv::Mat &frame) {
    Eigen::Quaterniond averageRotation(0, 0, 0, 0);
    Eigen::Vector3d averageTranslation(0, 0, 0);
    Eigen::Vector2d averageGeoTranslation(0, 0);
    FrameSnapshot snapshot;
    for (auto &pose:poseHistory) {
        auto poseChange = estimatePoseChange2D(pose, frame);
        auto rotation = poseChange.rotation * pose.rotation;
        // TODO: premultiply change by rotation
        // auto translation = poseChange.translation + pose.translation;
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
    }
    averageTranslation /= poseHistory.size();
    averageGeoTranslation /= poseHistory.size();
    averageRotation = Eigen::Quaterniond(
            averageRotation.w() / poseHistory.size(),
            averageRotation.x() / poseHistory.size(),
            averageRotation.y() / poseHistory.size(),
            averageRotation.z() / poseHistory.size()
    );

    // do we actually need this?
    // auto translationChange = averageTranslation - poseHistory.back().translation;
    // auto rotationChange = averageRotation * poseHistory.back().rotation.inverse();

    auto pose = Pose(averageRotation, averageTranslation, averageGeoTranslation, snapshot);
    poseHistory.emplace_back(pose);
    poseHistory.pop_front();

    path.emplace_back(pose.translation);
    geoPath.emplace_back(pose.geo);

    mapper.feed(pose, frame);

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

    std::vector<double> lat, lon;
    for (auto &point : geoPath) {
        lat.emplace_back(point.x());
        lon.emplace_back(point.y());
    }
    std::cout << geoPath.size() << std::endl;

    if (volumetric) {
        plt::plot3(x, y, z);
    } else {
        plt::plot(lat, lon);
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

    // TODO: estimatePoseChange3D is not used anyway
    Eigen::Vector2d geoTranslation(0, 0);

    return Pose(rotationDerivative, translationDifference, geoTranslation, snapshot);
}

Pose SLAM::estimatePoseChange2D(const Pose &from, const cv::Mat &frame) {
    auto[snapshot, cutoff] = makeSnapshot(frame, from.snapshot);
    if (cutoff) {
        return Pose(
                Eigen::Quaterniond(1, 0, 0, 0),
                Eigen::Vector3d(0, 0, 0),
                Eigen::Vector2d(0, 0),
                snapshot
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
    double distnaceToBottom = from.translation.z();
    double frameWidthMeters = frameWidthPx * pixelSize * (distnaceToBottom / focalLength - 1);
    double frameHeightMeters = frameHeightPx * pixelSize * (distnaceToBottom / focalLength - 1);
    auto translation2DMeters = Eigen::Vector2d(
            translation2DPx.x() / frameWidthPx * frameWidthMeters,
            translation2DPx.y() / frameHeightPx * frameHeightMeters
    );


    const double WGS84_BIG_RAD = 6378137;
    const double WGS84_SMALL_RAD = 6356752.3142;
    // assume translation is small
    // x -> north, y -> east
    double latitude = from.geo.x();
    double longitude = from.geo.y();
    double latitudeChange = -translation2DMeters.x() / WGS84_SMALL_RAD * 2 * M_PI;
    double longitudeChange = translation2DMeters.y() / WGS84_BIG_RAD * 2 * M_PI * cos(latitude);
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
            snapshot
    );
}