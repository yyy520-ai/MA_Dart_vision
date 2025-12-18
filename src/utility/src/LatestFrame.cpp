#include "LatestFrame.hpp"

// Provide global definitions for the externs declared in LatestFrame.hpp
std::shared_ptr<cv::Mat> latest_frame;
std::atomic<uint32_t> frames_produced{0};
std::atomic<uint32_t> frames_displayed{0};
