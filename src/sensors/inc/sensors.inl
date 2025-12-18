#include "sensors.hpp"
#include "LatestFrame.hpp"

template<class ...Args>
void sensors::BaseCamera::refreshImage(Args &&...args) {
    this->getImage();
    this->unpackArgs(args...);
}

// Overload to accept an additional LfStack<cv::Mat>& for direct display push
template<class ...Args>
void sensors::BaseCamera::refreshImage(LfStack<FromData> &fd, LfStack<cv::Mat> &display, Args &&...args) {
    this->getImage();
    // push to fd as usual
    fd.push(this->message_);
    // also push image directly to display stack for test
    if (!this->message_.img.empty()) {
        // avoid double clone: push a lightweight clone to display stack, but
        // publish a shared_ptr to the freshly captured image for the UI.
        display.push(this->message_.img.clone());
        auto mptr = std::make_shared<cv::Mat>(this->message_.img);
        std::atomic_store_explicit(&::latest_frame, mptr, std::memory_order_release);
        ::frames_produced.fetch_add(1, std::memory_order_relaxed);
    }
    // process remaining args if any
    this->unpackArgs(args...);
}

template<typename T, class ...Args>
void sensors::BaseCamera::unpackArgs(T &&t, Args &&...args) {
    if constexpr (std::is_same_v<T, LfStack<FromData>&>) {
        // Producer should only push the freshly captured message into the stack.
        // Earlier code popped then pushed which could remove data unexpectedly.
        t.push(this->message_);
        static int push_count = 0;
        if (push_count < 10) {
            std::cout << "sensors: pushed frame to fd_msgs (count=" << push_count << ") cols=" << this->message_.img.cols << " rows=" << this->message_.img.rows << std::endl;
        }
        ++push_count;
    } else {
        std::cout << "error type in sensors" << std::endl;
    }
    this->unpackArgs(args...);
}