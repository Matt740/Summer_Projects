#include <opencv2/opencv.hpp>
#include <iostream>
#include <cstdlib>   // for system()
#include <unistd.h>  // for access()

int main() {
    const std::string filename = "capture.jpg";

    // Capture image using libcamera-still
    std::string command = "libcamera-still -o " + filename + " -n --width 640 --height 480";
    int ret = system(command.c_str());

    if (ret != 0) {
        std::cerr << "ERROR: Failed to capture image with libcamera-still\n";
        return -1;
    }

    // Optional: wait for file to be fully written
    if (access(filename.c_str(), F_OK) != 0) {
        std::cerr << "ERROR: Captured file not found\n";
        return -1;
    }

    // Load the image with OpenCV
    cv::Mat frame = cv::imread(filename);
    if (frame.empty()) {
        std::cerr << "ERROR: Could not load image\n";
        return -1;
    }

    // Process or display
    cv::imshow("Captured Frame", frame);
    cv::waitKey(0);  // Wait for key press

    // Optionally save it again after processing
    cv::imwrite("processed_capture.jpg", frame);
    std::cout << "Saved processed image to processed_capture.jpg\n";

    return 0;
}
