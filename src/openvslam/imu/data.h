#ifndef OPENVSLAM_IMU_DATA_H
#define OPENVSLAM_IMU_DATA_H

#include "openvslam/type.h"

namespace openvslam {
namespace imu {

class data {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //! Constructor for scalar inputs
    data(const double acc_x, const double acc_y, const double acc_z,
         const double gyr_x, const double gyr_y, const double gyr_z,
         const double ts);

    //! Constructor for vector inputs
    data(const Vec3_t& acc, const Vec3_t& gyr, const double ts);

    //! acceleration [m/s^2]
    const Vec3_t acc_;
    //! gyroscope [rad/s]
    const Vec3_t gyr_;
    //! timestamp [s]
    const double ts_;
};

} // namespace imu
} // namespace openvslam

#endif // OPENVSLAM_IMU_DATA_H
