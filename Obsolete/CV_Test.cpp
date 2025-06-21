#include <opencv2/opencv.hpp>
#include <iostream>

int main() {
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Error: Cannot open camera." << std::endl;
        return -1;
    }

    cv::Mat frame, hsv, mask;

    cv::Scalar lower_hsv(0, 100, 100);
    cv::Scalar upper_hsv(10, 255, 255);

    while (true) {
        cap >> frame;
        if (frame.empty()) break;

        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, lower_hsv, upper_hsv, mask);

        cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
        cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            if (area > 500) {
                cv::Point2f center;
                float radius;
                cv::minEnclosingCircle(contour, center, radius);
                if (radius > 10) {
                    cv::circle(frame, center, (int)radius, cv::Scalar(0, 255, 0), 2);
                    cv::circle(frame, center, 3, cv::Scalar(0, 0, 255), -1);
                }
            }
        }

        cv::imshow("Ball Detection", frame);
        cv::imshow("Mask", mask);

        if (cv::waitKey(1) == 27) break;
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
