#include <iostream>

#include <opencv2/opencv.hpp>
#include "opencv2/opencv_modules.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "grpc_video_capture.h"
#include "slam.h"


int main() {
//    cv::VideoCapture capture = cv::VideoCapture("iss_fcpx.mp4");
//    cv::VideoCapture capture = cv::VideoCapture("mc.mov");
//    cv::VideoCapture capture = cv::VideoCapture("mcnd.mov");
//    cv::VideoCapture capture = cv::VideoCapture("amap.mov");
    cv::VideoCapture capture = cv::VideoCapture("txt.mov");
//    cv::VideoCapture capture = cv::VideoCapture("sample_mpg.avi");

    SLAM slam;

    std::mutex lock;
    cv::Mat frame;
    GrpcVideoCaptureImpl grpcVideoCapture([&](const cv::Mat &img) {
        lock.lock();
        if (frame.empty()) {
            frame = img;
        }
        lock.unlock();
    });
    grpc::EnableDefaultHealthCheckService(true);


    ServerBuilder builder;
    builder.AddListeningPort("0.0.0.0:50001", grpc::InsecureServerCredentials());
    builder.RegisterService(&grpcVideoCapture);
    auto server = builder.BuildAndStart();
    std::cout << "ready" << std::endl;

//    int x = 5;
    while (true) {
//        if (!capture.isOpened()) {
//            break;
//        }
//        capture.read(frame);

        lock.lock();
        if (!frame.empty()) {
            std::cout << "size: " << frame.channels() << std::endl;
            cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
            cv::resize(frame, frame, cv::Size(640, 480));
            slam.feed(frame);
//            x--;
//            if (x < 0) {
//                return 0;
//            }

            frame = cv::Mat();
        }
        lock.unlock();

        int keycode = cv::waitKey(10);
        if (keycode > 0) {
            std::cout << "keycode: " << keycode << std::endl;
            if (keycode == 115 || keycode == 50) // s || 2
                slam.savePathAsImage("path.png", false);
            if (keycode == 115 || keycode == 51) // s || 3
                slam.savePathAsImage("path3.png", true);
            if (keycode == 113) // q
                break;
        }
    }

    slam.savePathAsImage("path.png", false);
    slam.savePathAsImage("path3.png", true);

    return 0;
}
