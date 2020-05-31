#include <iostream>
#include <opencv2/opencv.hpp>
#include "opencv2/opencv_modules.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "slam.h"


int main() {
//    cv::VideoCapture capture = cv::VideoCapture("iss_fcpx.mp4");
//    cv::VideoCapture capture = cv::VideoCapture("mc.mov");
    cv::VideoCapture capture = cv::VideoCapture("mcnd.mov");
//    cv::VideoCapture capture = cv::VideoCapture("amap.mov");
//    cv::VideoCapture capture = cv::VideoCapture("txt.mov");
//    cv::VideoCapture capture = cv::VideoCapture("sample_mpg.avi");
    SLAM slam;

    cv::Mat frame;
    while (capture.isOpened()) {
        bool grabbed = capture.read(frame);

        if (!grabbed || frame.empty()) {
            break;
        }

        cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
        slam.feed(frame);

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
