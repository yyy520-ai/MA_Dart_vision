#include "mindvision/MindVisionCam.hpp"
#include <iostream>

using namespace sensors;

MindVisionCam::MindVisionCam(SerialPort &sp) : BaseCamera(sp) {}
MindVisionCam::~MindVisionCam() { cam_.close(); }

void MindVisionCam::cameraInit() {
    if (!cam_.open(0)) {
        std::cerr << "MindVision open failed: " << cam_.getLastError() << std::endl;
    } else {
        std::cout << "MindVision camera opened." << std::endl;
    }
}

void MindVisionCam::getImage() {
    cv::Mat img;
    if (cam_.grab(img)) {
        // 这里可将 img 送入 message_ 或其它处理流程
        // 示例：std::cout << "Grabbed MindVision frame: " << img.cols << "x" << img.rows << std::endl;
    } else {
        std::cerr << "MindVision grab failed: " << cam_.getLastError() << std::endl;
    }
}

void MindVisionCam::cameraModeChange(int now_mode, int my_color) {
    (void)now_mode;
    (void)my_color;
    // 可根据 now_mode/my_color 调用 cam_ 设置参数
}

void MindVisionCam::assertSuccess(uint32_t status) {
    if (status != 0) {
        std::cerr << "MindVision SDK error: " << status << std::endl;
    }
}
