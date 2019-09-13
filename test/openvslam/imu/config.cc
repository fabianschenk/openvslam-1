#include "openvslam/imu/config.h"
#include "openvslam/util/converter.h"

#include <gtest/gtest.h>

using namespace openvslam;

TEST(config, constructor) {
    // basic information
    const std::string name = "IMU";
    const double rate_hz = 250.0;
    const double rate_dt = 1.0 / rate_hz;

    // create the relative pose "from IMU to camera" (_ic) and "from camera to IMU" (_ci)
    const Mat33_t rel_rot_ic = util::converter::to_rot_mat(Vec3_t{0.707, 0.0, -0.707} * M_PI / 2.0);
    const Mat33_t rel_rot_ci = rel_rot_ic.transpose();
    const Vec3_t rel_trans_ic = Vec3_t{1.0, -2.0, 3.0};
    const Vec3_t rel_trans_ci = -rel_rot_ic.transpose() * rel_trans_ic;
    const Mat44_t rel_pose_ic = util::converter::to_eigen_cam_pose(rel_rot_ic, rel_trans_ic);
    const Mat44_t rel_pose_ci = util::converter::to_eigen_cam_pose(rel_rot_ci, rel_trans_ci);

    // acc noise [(m/s^2)/sqrt(Hz)]
    const double ns_acc = 0.04;
    // gyr noise [(rad/s]/sqrt(Hz)]
    const double ns_gyr = 0.02;
    // acc bias random walk [(m/s^3)/sqrt(Hz)]
    const double rw_acc_bias = 0.001;
    // gyr bias random walk [(rad/s^2]/sqrt(Hz)]
    const double rw_gyr_bias = 0.006;

    const auto cfg  = imu::config(name, rate_hz, rel_pose_ic, ns_acc, ns_gyr, rw_acc_bias, rw_gyr_bias);

    // check basic information
    EXPECT_EQ(cfg.get_name(), name);
    EXPECT_DOUBLE_EQ(cfg.get_rate_hz(), rate_hz);
    EXPECT_DOUBLE_EQ(cfg.get_rate_dt(), rate_dt);

    // check relative pose
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            EXPECT_NEAR(cfg.get_rel_pose_ci()(row, col), rel_pose_ci(row, col), 1e-10);
            EXPECT_NEAR(cfg.get_rel_pose_ic()(row, col), rel_pose_ic(row, col), 1e-10);
        }
    }
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            EXPECT_NEAR(cfg.get_rel_rot_ci()(row, col), rel_rot_ci(row, col), 1e-10);
            EXPECT_NEAR(cfg.get_rel_rot_ic()(row, col), rel_rot_ic(row, col), 1e-10);
        }
    }
    for (int row = 0; row < 3; ++row) {
        EXPECT_NEAR(cfg.get_rel_trans_ci()(row), rel_trans_ci(row), 1e-10);
        EXPECT_NEAR(cfg.get_rel_trans_ic()(row), rel_trans_ic(row), 1e-10);
    }

    // check covariance
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            if (row != col) {
                EXPECT_DOUBLE_EQ(cfg.get_acc_covariance()(row, col), 0.0);
                EXPECT_DOUBLE_EQ(cfg.get_gyr_covariance()(row, col), 0.0);
                EXPECT_DOUBLE_EQ(cfg.get_acc_bias_covariance()(row, col), 0.0);
                EXPECT_DOUBLE_EQ(cfg.get_gyr_bias_covariance()(row, col), 0.0);
            }
            else {
                EXPECT_DOUBLE_EQ(cfg.get_acc_covariance()(row, col), ns_acc * ns_acc * rate_hz);
                EXPECT_DOUBLE_EQ(cfg.get_gyr_covariance()(row, col), ns_gyr * ns_gyr * rate_hz);
                EXPECT_DOUBLE_EQ(cfg.get_acc_bias_covariance()(row, col), rw_acc_bias * rw_acc_bias * rate_hz);
                EXPECT_DOUBLE_EQ(cfg.get_gyr_bias_covariance()(row, col), rw_gyr_bias * rw_gyr_bias * rate_hz);
            }
        }
    }
}

TEST(config, covariance_update) {
    // basic information
    const std::string name = "IMU";
    const double rate_hz = 300.0;

    // create the relative pose "from IMU to camera" (_ic) and "from camera to IMU" (_ci)
    const Mat33_t rel_rot_ic = util::converter::to_rot_mat(Vec3_t{0.707, 0.0, -0.707} * M_PI / 2.0);
    const Vec3_t rel_trans_ic = Vec3_t{1.0, -2.0, 3.0};
    const Mat44_t rel_pose_ic = util::converter::to_eigen_cam_pose(rel_rot_ic, rel_trans_ic);

    // noise parameters
    const double ns_acc_1 = 0.04;
    const double ns_gyr_1 = 0.02;
    const double rw_acc_bias_1 = 0.001;
    const double rw_gyr_bias_1 = 0.006;

    auto cfg  = imu::config(name, rate_hz, rel_pose_ic, ns_acc_1, ns_gyr_1, rw_acc_bias_1, rw_gyr_bias_1);

    // check covariance
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            if (row != col) {
                EXPECT_DOUBLE_EQ(cfg.get_acc_covariance()(row, col), 0.0);
                EXPECT_DOUBLE_EQ(cfg.get_gyr_covariance()(row, col), 0.0);
                EXPECT_DOUBLE_EQ(cfg.get_acc_bias_covariance()(row, col), 0.0);
                EXPECT_DOUBLE_EQ(cfg.get_gyr_bias_covariance()(row, col), 0.0);
            }
            else {
                EXPECT_DOUBLE_EQ(cfg.get_acc_covariance()(row, col), ns_acc_1 * ns_acc_1 * rate_hz);
                EXPECT_DOUBLE_EQ(cfg.get_gyr_covariance()(row, col), ns_gyr_1 * ns_gyr_1 * rate_hz);
                EXPECT_DOUBLE_EQ(cfg.get_acc_bias_covariance()(row, col), rw_acc_bias_1 * rw_acc_bias_1 * rate_hz);
                EXPECT_DOUBLE_EQ(cfg.get_gyr_bias_covariance()(row, col), rw_gyr_bias_1 * rw_gyr_bias_1 * rate_hz);
            }
        }
    }

    // noise parameters
    const double ns_acc_2 = 0.02;
    const double ns_gyr_2 = 0.05;
    const double rw_acc_bias_2 = 0.003;
    const double rw_gyr_bias_2 = 0.008;

    cfg.set_acc_noise_density(ns_acc_2);
    cfg.set_gyr_noise_density(ns_gyr_2);
    cfg.set_acc_bias_random_walk(rw_acc_bias_2);
    cfg.set_gyr_bias_random_walk(rw_gyr_bias_2);

    // check covariance
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            if (row != col) {
                EXPECT_DOUBLE_EQ(cfg.get_acc_covariance()(row, col), 0.0);
                EXPECT_DOUBLE_EQ(cfg.get_gyr_covariance()(row, col), 0.0);
                EXPECT_DOUBLE_EQ(cfg.get_acc_bias_covariance()(row, col), 0.0);
                EXPECT_DOUBLE_EQ(cfg.get_gyr_bias_covariance()(row, col), 0.0);
            }
            else {
                EXPECT_DOUBLE_EQ(cfg.get_acc_covariance()(row, col), ns_acc_2* ns_acc_2 * rate_hz);
                EXPECT_DOUBLE_EQ(cfg.get_gyr_covariance()(row, col), ns_gyr_2 * ns_gyr_2 * rate_hz);
                EXPECT_DOUBLE_EQ(cfg.get_acc_bias_covariance()(row, col), rw_acc_bias_2 * rw_acc_bias_2 * rate_hz);
                EXPECT_DOUBLE_EQ(cfg.get_gyr_bias_covariance()(row, col), rw_gyr_bias_2 * rw_gyr_bias_2 * rate_hz);
            }
        }
    }
}
