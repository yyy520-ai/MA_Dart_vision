// CameraParams.hpp - centralized camera default parameters
#ifndef CAMERA_PARAMS_HPP_
#define CAMERA_PARAMS_HPP_

#include <cstdint>

namespace camera_params {
    // Exposure time in microseconds
    static constexpr float kDefaultExposureUs = 30000.0f;
    // Gain (SDK uses floating point representation)
    static constexpr float kDefaultGain = 16.0f;
    // ROI/default width/height used in setResolution call in HikRobot
    static constexpr int kRoiWidth = 384;
    static constexpr int kRoiWidthOffset = 300;
    static constexpr int kRoiHeight = 640;
    static constexpr int kRoiHeightOffset = 100;
    // Pixel format for the camera (Bayer RG8)
    static constexpr int kPixelFormat = static_cast<int>(PixelType_Gvsp_BayerRG8);
}

#endif // CAMERA_PARAMS_HPP_
