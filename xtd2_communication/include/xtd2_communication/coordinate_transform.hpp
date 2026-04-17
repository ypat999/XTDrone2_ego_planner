#ifndef XTD2_COMMUNICATION_COORDINATE_TRANSFORM_HPP
#define XTD2_COMMUNICATION_COORDINATE_TRANSFORM_HPP

#include <vector>
#include <array>
#include <cmath>
#include <Eigen/Dense>

namespace xtd2_communication {

class CoordinateTransform {
public:
    CoordinateTransform() = default;
    ~CoordinateTransform() = default;

    static std::vector<double> ned_to_enu_position(double x, double y, double z);
    static std::vector<double> enu_to_ned_position(double x, double y, double z);
    static std::vector<double> ned_to_enu_velocity(double vx, double vy, double vz);
    static std::vector<double> enu_to_ned_velocity(double vx, double vy, double vz);
    static std::vector<double> ned_to_enu_acceleration(double ax, double ay, double az);
    static std::vector<double> enu_to_ned_acceleration(double ax, double ay, double az);
    static std::vector<double> ned_to_enu_quaternion(double qw, double qx, double qy, double qz);
    static std::vector<double> enu_to_ned_quaternion(double qw, double qx, double qy, double qz);

    static std::vector<double> frd_to_flu_position(double x, double y, double z);
    static std::vector<double> flu_to_frd_position(double x, double y, double z);
    static std::vector<double> frd_to_flu_velocity(double vx, double vy, double vz);
    static std::vector<double> flu_to_frd_velocity(double vx, double vy, double vz);
    static std::vector<double> frd_to_flu_acceleration(double ax, double ay, double az);
    static std::vector<double> flu_to_frd_acceleration(double ax, double ay, double az);
    static std::vector<double> frd_to_flu_quaternion(double qw, double qx, double qy, double qz);
    static std::vector<double> flu_to_frd_quaternion(double qw, double qx, double qy, double qz);

    static Eigen::Matrix2d body_to_ned_matrix(double heading);
    static Eigen::Matrix2d ned_to_body_matrix(double heading);
    static Eigen::Matrix3d body_to_ned_3d_matrix(double heading);
    static Eigen::Matrix3d ned_to_body_3d_matrix(double heading);

    static std::vector<double> flu_to_ned_position(double flu_x, double flu_y, double flu_z, double heading);
    static std::vector<double> ned_to_flu_position(double ned_x, double ned_y, double ned_z, double heading);
    static std::vector<double> flu_to_ned_velocity(double flu_vx, double flu_vy, double flu_vz, double heading);
    static std::vector<double> ned_to_flu_velocity(double ned_vx, double ned_vy, double ned_vz, double heading);
    static std::vector<double> flu_to_ned_acceleration(double flu_ax, double flu_ay, double flu_az, double heading);
    static std::vector<double> ned_to_flu_acceleration(double ned_ax, double ned_ay, double ned_az, double heading);

    static std::vector<double> flu_to_frd_angular_velocity(double flu_wx, double flu_wy, double flu_wz);
    static std::vector<double> frd_to_flu_angular_velocity(double frd_wx, double frd_wy, double frd_wz);

    static std::vector<double> frd_to_ned_velocity(double vx, double vy, double vz, double heading);
    static std::vector<double> ned_to_frd_velocity(double vx, double vy, double vz, double heading);

    static std::vector<double> frd_ned_to_flu_enu_velocity(double frd_vx, double frd_vy, double frd_vz, double heading);
    static std::vector<double> flu_enu_to_frd_ned_velocity(double flu_vx, double flu_vy, double flu_vz, double heading);
    static std::vector<double> frd_ned_to_flu_enu_quaternion(double frd_qw, double frd_qx, double frd_qy, double frd_qz);
    static std::vector<double> flu_enu_to_frd_ned_quaternion(double flu_qw, double flu_qx, double flu_qy, double flu_qz);

    static double flu_to_ned_yawspeed(double flu_wz);
    static double ned_to_flu_yawspeed(double ned_wz);

    static std::vector<double> flu_to_enu_position(double flu_x, double flu_y, double flu_z, double heading);
    static std::vector<double> enu_to_flu_position(double enu_x, double enu_y, double enu_z, double heading);
    static std::vector<double> flu_to_enu_velocity(double flu_vx, double flu_vy, double flu_vz, double heading);
    static std::vector<double> enu_to_flu_velocity(double enu_vx, double enu_vy, double enu_vz, double heading);
    static std::vector<double> flu_to_enu_acceleration(double flu_ax, double flu_ay, double flu_az, double heading);
    static std::vector<double> enu_to_flu_acceleration(double enu_ax, double enu_ay, double enu_az, double heading);

    static Eigen::Vector3d flu_to_enu_vector_by_rotation_matrix(const Eigen::Vector3d& flu_vec, const Eigen::Matrix3d& rotation_matrix);
    static Eigen::Vector3d enu_to_flu_vector_by_rotation_matrix(const Eigen::Vector3d& enu_vec, const Eigen::Matrix3d& rotation_matrix);
    static Eigen::Vector3d flu_to_ned_vector_by_rotation_matrix(const Eigen::Vector3d& flu_vec, const Eigen::Matrix3d& rotation_matrix);

    static std::tuple<double, double, double> flu_to_ned_position_px4_odom(
        double flu_x, double flu_y, double flu_z,
        double init_n, double init_e, double init_d, double init_heading);
    static std::vector<double> flu_to_ned_quaternion_px4_odom(
        double flu_qw, double flu_qx, double flu_qy, double flu_qz, double init_heading);

    static std::vector<double> qmult(double q1_w, double q1_x, double q1_y, double q1_z,
                                      double q2_w, double q2_x, double q2_y, double q2_z);
    static Eigen::Matrix3d quat2mat(double w, double x, double y, double z);
    static std::tuple<std::vector<double>, std::vector<double>> inverse_transform(
        double pos_x, double pos_y, double pos_z,
        double quat_w, double quat_x, double quat_y, double quat_z);
    static double heading_from_quaternion(double w, double x, double y, double z);
    static Eigen::Matrix3d create_rotation_matrix_from_quaternion(double w, double x, double y, double z);

    static std::tuple<std::vector<double>, std::vector<double>> multiply_transforms(
        double t1_x, double t1_y, double t1_z, double t1_qw, double t1_qx, double t1_qy, double t1_qz,
        double t2_x, double t2_y, double t2_z, double t2_qw, double t2_qx, double t2_qy, double t2_qz);

private:
    static Eigen::Matrix3d quat_to_rot(double w, double x, double y, double z);
    static std::vector<double> rot_to_quat(const Eigen::Matrix3d& R);
};

} 

#endif
