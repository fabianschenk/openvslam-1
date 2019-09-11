#include "openvslam/imu/data.h"

#include <gtest/gtest.h>

using namespace openvslam;

TEST(data, constructor_1) {
    const double acc_x = 1.5964;
    const double acc_y = 2.3451;
    const double acc_z = -4.4643;
    const double gyr_x = 5.1235;
    const double gyr_y = -7.1243;
    const double gyr_z = 3.2463;
    const double ts = 245692394.23523;

    const auto data = imu::data(acc_x, acc_y, acc_z, gyr_x, gyr_y, gyr_z, ts);

    EXPECT_DOUBLE_EQ(acc_x, data.acc_(0));
    EXPECT_DOUBLE_EQ(acc_y, data.acc_(1));
    EXPECT_DOUBLE_EQ(acc_z, data.acc_(2));
    EXPECT_DOUBLE_EQ(gyr_x, data.gyr_(0));
    EXPECT_DOUBLE_EQ(gyr_y, data.gyr_(1));
    EXPECT_DOUBLE_EQ(gyr_z, data.gyr_(2));
    EXPECT_DOUBLE_EQ(ts, data.ts_);
}

TEST(data, constructor_2) {
    const Vec3_t acc{1.4786, -4.4641, 9.2464};
    const Vec3_t gyr{-4.5634, -1.8974, 2.3456};
    const double ts = 7549532531.46344;

    const auto data = imu::data(acc, gyr, ts);

    for (unsigned int i = 0; i < 3; ++i) {
        EXPECT_DOUBLE_EQ(acc(i), data.acc_(i));
        EXPECT_DOUBLE_EQ(gyr(i), data.gyr_(i));
    }
    EXPECT_DOUBLE_EQ(ts, data.ts_);
}
