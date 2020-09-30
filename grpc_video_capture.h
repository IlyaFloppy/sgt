#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <functional>
#include <utility>

#include "protos/video.pb.h"
#include "protos/video.grpc.pb.h"
#include "grpc++/grpc++.h"


using namespace grpc;

using FrameConsumer = std::function<void(cv::Mat)>;

class GrpcVideoCaptureImpl final : public GrpcVideoCapture::Service {
private:
    int imageNumber = -1;
public:
    FrameConsumer consumer;

    explicit GrpcVideoCaptureImpl(FrameConsumer consumer) :
            GrpcVideoCapture::Service(),
            consumer(std::move(consumer)) {}

    Status Post(ServerContext *context, const VideoFrame *request, VoidEntity *reply) override {
        const auto &strdata = request->data();
        std::vector<char> data(strdata.begin(), strdata.end());
        auto dataMat = cv::Mat(data, false);
        auto image = cv::imdecode(data, cv::IMREAD_COLOR);

        if (imageNumber > 0) {
            std::stringstream ss;
            ss << "chessboard/" << imageNumber << ".png";
            cv::imwrite(ss.str(), image);
            imageNumber++;
        }
        consumer(image);

        reply->set_message("ok");
        return Status::OK;
    }

};