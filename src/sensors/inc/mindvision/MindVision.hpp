#pragma once
#include <CameraApi.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

class MindVisionCamera {
public:
    MindVisionCamera();
    ~MindVisionCamera();
    bool open(int deviceIndex = 0);
    void close();
    bool grab(cv::Mat& image);
    bool isOpened() const;
    std::string getLastError() const;
private:
    int m_hCamera = -1;
    tSdkCameraCapbility m_capability;
    unsigned char* m_pBuffer = nullptr;
    tSdkFrameHead m_frameHead;
    bool m_opened = false;
    std::string m_lastError;
    int m_channel = 3; // 默认彩色
    int m_width = 0;
    int m_height = 0;
};
