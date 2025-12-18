#ifndef MINDVISIONCAM_HPP_
#define MINDVISIONCAM_HPP_

#include "sensors.hpp"
#include <mindvision/MindVision.hpp>

namespace sensors {
class MindVisionCam : public virtual BaseCamera {
public:
    explicit MindVisionCam(SerialPort &sp);
    ~MindVisionCam();
    void cameraInit() override;
private:
    void getImage() override;
    void cameraModeChange(int now_mode, int my_color) override;
    void assertSuccess(uint32_t status) override;
    MindVisionCamera cam_;
};
}

#endif // MINDVISIONCAM_HPP_
