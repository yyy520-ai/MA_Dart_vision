
#ifndef LATEST_FRAME_HPP_
#define LATEST_FRAME_HPP_

#include <memory>
#include <opencv2/opencv.hpp>
#include <atomic>

// latest_frame is accessed lock-free using std::atomic_load/atomic_store
extern std::shared_ptr<cv::Mat> latest_frame;
extern std::atomic<uint32_t> frames_produced;
extern std::atomic<uint32_t> frames_displayed;

#endif // LATEST_FRAME_HPP_

