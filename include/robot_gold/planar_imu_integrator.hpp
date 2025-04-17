#include <iostream>
#include <cmath>
#include <numeric>
#include <vector>

namespace algorithms {

    class PlanarImuIntegrator {
    public:

        PlanarImuIntegrator();
        //~PlanarImuIntegrator() = default;

        // TODO: Call this regularly to integrate gyro_z over time
        void update(float gyro_z, double dt);

        // TODO: Calibrate the gyroscope by computing average from static samples
        void setCalibration(std::vector<float> gyro);

        // TODO: Return the current estimated yaw
        [[nodiscard]] float getYaw() const;

        // TODO: Reset orientation and calibration
        void reset();

    private:
        float theta_;       // Integrated yaw angle (radians)
        float gyro_offset_; // Estimated gyro bias
    };
}