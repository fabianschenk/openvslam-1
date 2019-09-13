#ifndef OPENVSLAM_IMU_CONFIG_H
#define OPENVSLAM_IMU_CONFIG_H

#include "openvslam/type.h"

namespace openvslam {
namespace imu {

class config {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    //! Constructor
    config(const std::string& name, const double rate_hz, const Mat44_t& rel_pose_ic,
           const double ns_acc, const double ns_gyr, const double rw_acc_bias, const double rw_gyr_bias);

    //---------------------------
    // Setters and Getters

    //! Get IMU model name
    std::string get_name() const;
    //! Get IMU rate [Hz]
    double get_rate_hz() const;
    //! Get IMU rate [s]
    double get_rate_dt() const;

    //! Get IMU's relative pose w.r.t. the camera
    Mat44_t get_rel_pose_ic() const;
    //! Get IMU's relative rotation w.r.t. the camera
    Mat33_t get_rel_rot_ic() const;
    //! Get IMU's relative translation w.r.t. the camera
    Vec3_t get_rel_trans_ic() const;
    //! Get camera's relative pose w.r.t. the IMU
    Mat44_t get_rel_pose_ci() const;
    //! Get camera's relative rotation w.r.t. the IMU
    Mat33_t get_rel_rot_ci() const;
    //! Get camera's relative translation w.r.t. the IMU
    Vec3_t get_rel_trans_ci() const;

    //! Set acceleration noise density [m/s^2/sqrt(Hz)]
    void set_acc_noise_density(const double ns_acc);
    //! Set gyroscope noise density [rad/s/sqrt(Hz)]
    void set_gyr_noise_density(const double ns_gyr);
    //! Set random walk of acceleration sensor bias [m/s^3/sqrt(Hz)]
    void set_acc_bias_random_walk(const double rw_acc_bias);
    //! Set random walk of gyroscope sensor bias [rad/s^2/sqrt(Hz)]
    void set_gyr_bias_random_walk(const double rw_gyr_bias);

    //! Get acceleration covariance [(m/s^2)^2]
    Mat33_t get_acc_covariance() const;
    //! Get gyroscope covariance [(rad/s)^2]
    Mat33_t get_gyr_covariance() const;
    //! Get acceleration bias covariance [(m/s^3)^2]
    Mat33_t get_acc_bias_covariance() const;
    //! Get gyroscope bias covariance [(rad/s^2)^2]
    Mat33_t get_gyr_bias_covariance() const;

private:
    //! Update rel_pose_ci_ using rel_pose_ic_
    void update_pose();
    //! Update covariances using the currently assigned variables
    void update_covariance();

    //! IMU model name
    std::string name_;
    //! IMU rate [Hz]
    double rate_hz_;
    //! IMU rate [s]
    double rate_dt_;

    //! IMU's relative pose w.r.t the camera
    Mat44_t rel_pose_ic_;
    //! camera's relative pose w.r.t the IMU
    Mat44_t rel_pose_ci_;

    //! covariance of acceleration sensor [(m/s^2)^2] = [m/s^2/sqrt(Hz)] * [m/s^2/sqrt(Hz)] * [Hz]
    Mat33_t cov_acc_ = Mat33_t::Identity() ;
    //! covariance of gyroscope sensor [(rad/s)^2] = [rad/s/sqrt(Hz)] * [rad/s/sqrt(Hz)] * [Hz]
    Mat33_t cov_gyr_ = Mat33_t::Identity() ;
    //! covariance gy acceleration sensor bias [(m/s^3)^2] = [m/s^3/sqrt(Hz)] * [m/s^3/sqrt(Hz)] * [Hz]
    Mat33_t cov_acc_bias_ = Mat33_t::Identity() ;
    //! covariance of gyroscope sensor bias [(rad/s^2)^2] = [rad/s^2/sqrt(Hz)] * [rad/s^2/sqrt(Hz)] * [Hz]
    Mat33_t cov_gyr_bias_ = Mat33_t::Identity() ;

    //! noise density of acceleration sensor [m/s^2/sqrt(Hz)]
    double ns_acc_;
    //! noise density of gyroscope sensor [rad/s/sqrt(Hz)]
    double ns_gyr_;
    //! random walk of acceleration sensor bias [m/s^3/sqrt(Hz)]
    double rw_acc_bias_;
    //! random walk of gyroscope sensor bias [rad/s^2/sqrt(Hz)]
    double rw_gyr_bias_;
};

} // namespace imu
} // namespace openvslam

#endif // OPENVSLAM_IMU_CONFIG_H
