#include "openvslam/imu/data.h"

namespace openvslam {
namespace imu {

data::data(const double acc_x, const double acc_y, const double acc_z,
           const double gyr_x, const double gyr_y, const double gyr_z,
           const double ts)
        : acc_(acc_x, acc_y, acc_z), gyr_(gyr_x, gyr_y, gyr_z), ts_(ts) {}

data::data(const Vec3_t& acc, const Vec3_t& gyr, const double ts)
        : acc_(acc), gyr_(gyr), ts_(ts) {}

} // namespace imu
} // namespace openvslam
