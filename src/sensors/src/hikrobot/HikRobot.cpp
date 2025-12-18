#include "hikrobot/HikRobot.hpp"

#include <opencv2/opencv.hpp>
#include <thread>

#include "LatestFrame.hpp"

#include "function/function.hpp"
#include "../inc/CameraParams.hpp"

HikRobot::HikRobot(SerialPort &sp) : sensors::BaseCamera(sp) {
    this->error_ = function::getNowTimestamp();
}

HikRobot::~HikRobot() {
    this->closeCamera();
}

void HikRobot::cameraInit() {
    auto n_ret = MV_OK;
    this->camera_size_ = 0;
    memset(&this->stDeviceList_, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

    n_ret = MV_CC_EnumDevices(MV_USB_DEVICE, &this->stDeviceList_);

    if (n_ret != MV_OK) {
        printf("MV_CC_EnumDevices fail for n_ret = [%x].\n", n_ret);
        return;
    }

    if (this->stDeviceList_.nDeviceNum > 0) {
        for (size_t i = 0 ; i < this->stDeviceList_.nDeviceNum ; ++i) {
            this->pDeviceInfo_ = this->stDeviceList_.pDeviceInfo[i];

            if (this->pDeviceInfo_ == nullptr) {
                break;
            }

            if (!this->createHandle()) {
                continue;
            }

            if (!this->openCamera()) {
                continue;
            }

            printf("[HikRobot Debug] setEnumValue TriggerMode = %d\n", MV_TRIGGER_MODE_OFF);
            this->setEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF);
            // read pixel format from JSON (fallback to compiled default)
            int pixel_fmt = static_cast<int>(camera_params::kPixelFormat);
            if (!J_CAMERA.config_["hikrobot"]["pixel_format"].empty()) {
                J_CAMERA.config_["hikrobot"]["pixel_format"] >> pixel_fmt;
            }
            printf("[HikRobot Debug] setEnumValue PixelFormat = %d\n", pixel_fmt);
            this->setEnumValue("PixelFormat", static_cast<unsigned int>(pixel_fmt));
            // printf("[HikRobot Debug] setEnumValue ADCBitDepth = %d\n", 0);
            // this->setEnumValue("ADCBitDepth", 0);
            // disable frame-rate limiting so we can increase exposure
            printf("[HikRobot Debug] setBoolValue AcquisitionFrameRateEnable = %d\n", 0);
            this->setBoolValue("AcquisitionFrameRateEnable", false);
            // set large exposure and gain to make image brighter by default
            float desired_exposure = camera_params::kDefaultExposureUs; // microseconds
            float desired_gain = camera_params::kDefaultGain; // max
            if (!J_CAMERA.config_["hikrobot"]["exposure_us"].empty()) {
                J_CAMERA.config_["hikrobot"]["exposure_us"] >> desired_exposure;
            }
            if (!J_CAMERA.config_["hikrobot"]["gain"].empty()) {
                J_CAMERA.config_["hikrobot"]["gain"] >> desired_gain;
            }
            this->setFloatValue("ExposureTime", desired_exposure);
            this->setFloatValue("Gain", desired_gain);//0~16
            int roi_w = camera_params::kRoiWidth;
            int roi_woff = camera_params::kRoiWidthOffset;
            int roi_h = camera_params::kRoiHeight;
            int roi_hoff = camera_params::kRoiHeightOffset;
            if (!J_CAMERA.config_["hikrobot"]["roi"]["width"].empty()) J_CAMERA.config_["hikrobot"]["roi"]["width"] >> roi_w;
            if (!J_CAMERA.config_["hikrobot"]["roi"]["width_offset"].empty()) J_CAMERA.config_["hikrobot"]["roi"]["width_offset"] >> roi_woff;
            if (!J_CAMERA.config_["hikrobot"]["roi"]["height"].empty()) J_CAMERA.config_["hikrobot"]["roi"]["height"] >> roi_h;
            if (!J_CAMERA.config_["hikrobot"]["roi"]["height_offset"].empty()) J_CAMERA.config_["hikrobot"]["roi"]["height_offset"] >> roi_hoff;
            this->setResolution(-1, roi_w, roi_woff, -1, roi_h, roi_hoff);
            if (!this->grapImage()) {
                return;
            }
            // read back actual values and log
            float actual_exp = this->getFloatValue("ExposureTime");
            float actual_gain = this->getFloatValue("Gain");
            std::cout << "[HikRobot Debug] requested ExposureTime=" << desired_exposure << " Gain=" << desired_gain
                      << " actual ExposureTime=" << actual_exp << " Gain=" << actual_gain << std::endl;
            ++this->camera_size_;
        }
    } else {
        printf("No Devices Found.\n");
        return;
    }

    printf("Initialize %d camera(s).\n", this->camera_size_);

}

bool HikRobot::createHandle() {
    auto n_ret = MV_CC_CreateHandle(&this->handle_, this->pDeviceInfo_);

    if (n_ret != MV_OK) {
        printf("MV_CC_CreateHandle fail for n_ret = [%x].\n", n_ret);
        return false;
    }

    return true;
}

bool HikRobot::openCamera() {
    if (this->handle_ == nullptr) {
        return false;
    }

    auto n_ret = MV_CC_OpenDevice(this->handle_);

    if (n_ret != MV_OK) {
        printf("MV_CC_OpenDevice fail for n_ret = [%x].\n", n_ret);
        return false;
    }

    return true;
}

void HikRobot::closeCamera() {
    auto n_ret = MV_CC_StopGrabbing(this->handle_);
    if (n_ret != MV_OK) {
        printf("MV_CC_StopGrabbing fail for n_ret = [%x].\n", n_ret);
    }

    n_ret = MV_CC_CloseDevice(this->handle_);
    if (n_ret != MV_OK) {
        printf("MV_CC_CloseDevice fail for n_ret = [%x].\n", n_ret);
    }

    n_ret = MV_CC_DestroyHandle(this->handle_);
    if (n_ret != MV_OK) {
        printf("MV_CC_DestroyHandle fail for n_ret = [%x].\n", n_ret);
    }
}

unsigned int HikRobot::getIntValue(const char *key) {
    MVCC_INTVALUE value;
    auto n_ret = MV_CC_GetIntValue(this->handle_, key, &value);

    if (n_ret != MV_OK) {
        printf("MV_CC_GetIntValue fail for n_ret = [%x].\n", n_ret);
        return 0;
    }

    return value.nCurValue;
}

float HikRobot::getFloatValue(const char *key) {
    MVCC_FLOATVALUE value;
    auto n_ret = MV_CC_GetFloatValue(this->handle_, key, &value);

    if (n_ret != MV_OK) {
        printf("MV_CC_GetFloatValue fail for n_ret = [%x].\n", n_ret);
        return 0;
    }

    return value.fCurValue;
}

unsigned int HikRobot::getEnumValue(const char *key) {
    MVCC_ENUMVALUE value;
    auto n_ret = MV_CC_GetEnumValue(this->handle_, key, &value);

    if (n_ret != MV_OK) {
        printf("MV_CC_GetEnumValue fail for n_ret = [%x].\n", n_ret);
        return 0;
    }

    return value.nCurValue;
}

bool HikRobot::getBoolValue(const char *key) {
    bool value;
    auto n_ret = MV_CC_GetBoolValue(this->handle_, key, &value);

    if (n_ret != MV_OK) {
        printf("MV_CC_GetBoolValue fail for n_ret = [%x].\n", n_ret);
        return false;
    }

    return value;
}

MVCC_STRINGVALUE HikRobot::getStringValue(const char *key) {
    MVCC_STRINGVALUE value;
    auto n_ret = MV_CC_GetStringValue(this->handle_, key, &value);

    if (n_ret != MV_OK) {
        printf("MV_CC_GetStringValue fail for n_ret = [%x].\n", n_ret);
        return value;
    }

    return value;
}

void HikRobot::setTriggerMode() {
    auto n_ret = MV_CC_SetEnumValue(this->handle_, "TriggerMode", 0);

    if (n_ret != MV_OK) {
        printf("MV_CC_SetEnumValue fail for n_ret = [%x].\n", n_ret);
    }
}

void HikRobot::setIntValue(const char *key, unsigned int value) {
    auto n_ret = MV_CC_SetIntValue(this->handle_, key, value);

    if (n_ret != MV_OK) {
        printf("MV_CC_SetIntValue fail for n_ret = [%x].\n", n_ret);
    }
}

void HikRobot::setFloatValue(const char *key, float value) {
    auto n_ret = MV_CC_SetFloatValue(this->handle_, key, value);

    if (n_ret != MV_OK) {
        printf("MV_CC_SetFloatValue fail for n_ret = [%x].\n", n_ret);
    }
}

void HikRobot::setEnumValue(const char *key, unsigned int value) {
    auto n_ret = MV_CC_SetEnumValue(this->handle_, key, value);

    if (n_ret != MV_OK) {
        printf("MV_CC_SetEnumValue fail for n_ret = [%x].\n", n_ret);
    }
}

void HikRobot::setBoolValue(const char *key, bool value) {
    auto n_ret = MV_CC_SetBoolValue(this->handle_, key, value);

    if (n_ret != MV_OK) {
        printf("MV_CC_SetBoolValue fail for n_ret = [%x].\n", n_ret);
    }
}

void HikRobot::setResolution(
        int max_width,
        int roi_width,
        int roi_width_offset,
        int max_height,
        int roi_height,
        int roi_height_offset) {
    auto n_ret = MV_OK;
    if (this->camera_state_ >= Camera_State::GRABBING) {
        printf("Stop grabbing first.\n");

        n_ret = MV_CC_StopGrabbing(this->handle_);

        cv::waitKey(200);
    }

    if (n_ret != MV_OK) {
        printf("MV_CC_StopGrabbing fail for n_ret = [%x].\n", n_ret);
        return;
    }

    (void)max_width;
    (void)max_height;
    this->setIntValue("Width", roi_width);
    this->setIntValue("OffsetX", roi_width_offset);

    this->setIntValue("Height", roi_height);
    this->setIntValue("OffsetY", roi_height_offset);

    if (this->camera_state_ >= Camera_State::GRABBING) {
        this->grapImage();
    }
}

bool HikRobot::grapImage() {
    auto n_ret = MV_CC_StartGrabbing(this->handle_);
    if (n_ret != MV_OK) {
        printf("MV_CC_StartGrabbing fail for n_ret = [%x].\n", n_ret);
        return false;
    }
    std::cout << "HikRobot: MV_CC_StartGrabbing returned " << std::hex << n_ret << std::dec << std::endl;

    memset(&this->stParam_, 0, sizeof(MVCC_INTVALUE));
    n_ret = MV_CC_GetIntValue(this->handle_, "PayloadSize", &this->stParam_);
    if (n_ret != MV_OK) {
        printf("MV_CC_GetIntValue fail for n_ret = [%x].\n", n_ret);
        return false;
    }


    this->nDataSize_ = this->stParam_.nCurValue;

    this->camera_state_ = Camera_State::GRABBING;

    return true;
}

void HikRobot::cameraModeChange(int now_mode, int my_color) {
    if ((now_mode == 1 || now_mode == 0) && (now_mode != this->pre_mode_ || my_color != this->pre_my_color_)) {
        if (my_color == 0) {
            std::cout << "Detect Red." << std::endl;
            float val = J_DETECT.config_["camera"]["daheng"]["exposure"]["red_exposure"];
            std::cout << "val = " << val << std::endl;
            this->setFloatValue("ExposureTime", val);
        } else if (my_color == 1) {
            std::cout << "Detect Blue." << std::endl;
            float val = J_DETECT.config_["camera"]["daheng"]["exposure"]["blue_exposure"];
            std::cout << "val = " << val << std::endl;
            this->setFloatValue("ExposureTime", val);
        }
    }
    this->pre_mode_ = now_mode;
    this->pre_my_color_ = my_color;
}

void HikRobot::assertSuccess(uint32_t status) {
    if (status != MV_OK) {
        printf("Camera_Error: %x\n", status);
        this->closeCamera();
        return;
    }
}

void HikRobot::getImage() {
    // debug: getImage entry logged only when verbose debugging is enabled
    // Try to receive CarData from serial for a short period; if serial
    // is unavailable or no data arrives, fall back to a default CarData
    // so camera capture and detection can run for development/testing.
    // Try to receive CarData from serial once; if unavailable, quickly fall back
    // to default CarData so camera capture is not blocked by serial I/O.
    CarData car_data;
    if (!this->sp_.receive(car_data)) {
        static bool warned = false;
        if (!warned) {
            std::cout << "HikRobot: serial CarData not available, using default CarData for testing." << std::endl;
            warned = true;
        }
        // fill reasonable defaults
        car_data.header = 0xA5;
        car_data.mode = 1; // enable auto-detect/record by default for testing
        car_data.status = 0;
        car_data.number = 0;
        car_data.dune = 0;
        car_data.CRC16 = 0;
    }
//    this->cameraModeChange(car_data.mode, car_data.my_color);

    this->message_.cd = car_data;
    this->message_.timestamp = function::getNowTimestamp();
    this->error_ = function::getNowTimestamp();
    memset(&this->stOutFrame_, 0, sizeof(MV_FRAME_OUT));
    int wait_count = 0;
    int n_ret = MV_OK;
    const int timeout_ms = 200; // shorter timeout for diagnostics
    while (true) {
        n_ret = MV_CC_GetImageBuffer(this->handle_, &this->stOutFrame_, timeout_ms);
        if (n_ret == MV_OK) break;
        this->message_.timestamp = function::getNowTimestamp();
        auto minus = function::timestampMinus(this->message_.timestamp, this->error_) / 1e3;
        if ((++wait_count % 50) == 0) {
            std::cout << "HikRobot: still waiting for image buffer, attempts=" << wait_count << std::endl;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        if (minus > (double)J_SENSORS.config_["camera"]["drop_time"]) {
            exit(0);
        }
    }
    static int _ok_count = 0;
    if ((++_ok_count % 100) == 0) {
        std::cout << "HikRobot: MV_CC_GetImageBuffer OK (count=" << _ok_count << ")" << std::endl;
    }
    // Diagnostic: print frame buffer info right after acquisition
    std::cout << "HikRobot: stOutFrame width=" << this->stOutFrame_.stFrameInfo.nWidth
              << " height=" << this->stOutFrame_.stFrameInfo.nHeight
              << " payload=" << this->stOutFrame_.stFrameInfo.nFrameLen
              << " pBufAddr=" << (this->stOutFrame_.pBufAddr ? "ok" : "null")
              << std::endl;

    if (this->stOutFrame_.stFrameInfo.enPixelType == PixelType_Gvsp_BayerRG8 && this->stOutFrame_.pBufAddr != nullptr) {

        // allocate or resize pBGR_ only when needed to avoid per-frame new/delete
        size_t needed = this->stOutFrame_.stFrameInfo.nWidth * this->stOutFrame_.stFrameInfo.nHeight * 4 + 2048;
        if (!this->pBGR_ || this->pBGR_size_ < needed) {
            delete[] this->pBGR_;
            this->pBGR_ = new unsigned char[needed];
            this->pBGR_size_ = needed;
        }

        MV_CC_PIXEL_CONVERT_PARAM stConvertParam;
        stConvertParam.nWidth = this->stOutFrame_.stFrameInfo.nWidth;
        stConvertParam.nHeight = this->stOutFrame_.stFrameInfo.nHeight;
        stConvertParam.pSrcData = this->stOutFrame_.pBufAddr;
        stConvertParam.nSrcDataLen = this->stOutFrame_.stFrameInfo.nFrameLen;
        stConvertParam.enSrcPixelType = this->stOutFrame_.stFrameInfo.enPixelType;
        stConvertParam.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
        stConvertParam.pDstBuffer = this->pBGR_;
        stConvertParam.nDstBufferSize = this->stOutFrame_.stFrameInfo.nWidth * this->stOutFrame_.stFrameInfo.nHeight * 4 + 2048;
        auto n_ret = MV_CC_ConvertPixelType(this->handle_, &stConvertParam);

        if (n_ret != MV_OK) {
            printf("MV_CC_ConvertPixelType fail for n_ret = [%x].\n", n_ret);
            return;
        }
        std::cout << "HikRobot: MV_CC_ConvertPixelType returned 0x" << std::hex << n_ret << std::dec << std::endl;

    this->message_.img = cv::Mat(this->stOutFrame_.stFrameInfo.nHeight, this->stOutFrame_.stFrameInfo.nWidth, CV_8UC3, this->pBGR_).clone();
    cv::transpose(this->message_.img, this->message_.img);
    cv::flip(this->message_.img, this->message_.img, -1);
    cv::flip(this->message_.img, this->message_.img, 1);
    MV_CC_FreeImageBuffer(this->handle_, &this->stOutFrame_);
    std::cout << "HikRobot: captured frame size=" << this->message_.img.cols << "x" << this->message_.img.rows
              << " payload=" << this->nDataSize_ << " format=" << this->stOutFrame_.stFrameInfo.enPixelType << std::endl;

    // print intensity range
    cv::Mat gray;
    cv::cvtColor(this->message_.img, gray, cv::COLOR_BGR2GRAY);
    double minv, maxv;
    cv::minMaxLoc(gray, &minv, &maxv);
    std::cout << "HikRobot: frame min=" << minv << " max=" << maxv << std::endl;

    // debug: frame saving disabled by default to avoid per-frame disk I/O

    // update global latest_frame for UI display (lock-free)
    {
        auto mptr = std::make_shared<cv::Mat>(this->message_.img);
        std::atomic_store_explicit(&::latest_frame, mptr, std::memory_order_release);
        ::frames_produced.fetch_add(1, std::memory_order_relaxed);
    }

    } else {
        printf("Unsupported PixelType.\n");
        return;
    }
}




