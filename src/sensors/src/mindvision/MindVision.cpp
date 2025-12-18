#include "mindvision/MindVision.hpp"
#include <cstring>

MindVisionCamera::MindVisionCamera() {}

MindVisionCamera::~MindVisionCamera() {
    close();
}

bool MindVisionCamera::open(int deviceIndex) {
    int cameraNums = 0;
    tSdkCameraDevInfo cameraList[8];
    int sdkInitStatus = CameraSdkInit(1);
    std::cerr << "CameraSdkInit status: " << sdkInitStatus << std::endl;
    int enumStatus = CameraEnumerateDevice(cameraList, &cameraNums);
    std::cerr << "CameraEnumerateDevice status: " << enumStatus << ", found: " << cameraNums << std::endl;
    if (cameraNums <= deviceIndex) {
        m_lastError = "No MindVision camera found.";
        return false;
    }
    if (CameraInit(&cameraList[deviceIndex], -1, -1, &m_hCamera) != CAMERA_STATUS_SUCCESS) {
        m_lastError = "CameraInit failed.";
        return false;
    }
    CameraGetCapability(m_hCamera, &m_capability);
    CameraPlay(m_hCamera);
    tSdkImageResolution* pImageSize = m_capability.pImageSizeDesc;
    m_width = pImageSize->iWidth;
    m_height = pImageSize->iHeight;
    m_opened = true;
    return true;
}

void MindVisionCamera::close() {
    if (m_opened) {
        CameraUnInit(m_hCamera);
        m_opened = false;
    }
    if (m_pBuffer) {
        CameraAlignFree(m_pBuffer);
        m_pBuffer = nullptr;
    }
}

bool MindVisionCamera::grab(cv::Mat& image) {
    if (!m_opened) return false;
    unsigned char* pRawBuffer = nullptr;
    int status = CameraGetImageBuffer(m_hCamera, &m_frameHead, &pRawBuffer, 100);
    if (status != CAMERA_STATUS_SUCCESS) {
        m_lastError = "CameraGetImageBuffer failed.";
        return false;
    }
    if (!m_pBuffer) {
        m_pBuffer = (unsigned char*)CameraAlignMalloc(m_frameHead.iWidth * m_frameHead.iHeight * m_channel, 16);
    }
    CameraImageProcess(m_hCamera, pRawBuffer, m_pBuffer, &m_frameHead);
    CameraReleaseImageBuffer(m_hCamera, pRawBuffer);
    image = cv::Mat(m_frameHead.iHeight, m_frameHead.iWidth, CV_8UC3, m_pBuffer).clone();
    return true;
}

bool MindVisionCamera::isOpened() const {
    return m_opened;
}

std::string MindVisionCamera::getLastError() const {
    return m_lastError;
}
