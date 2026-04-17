#include "xtd2_communication/coordinate_transform.hpp"
#include <algorithm>

namespace xtd2_communication {

Eigen::Matrix3d CoordinateTransform::quat_to_rot(double w, double x, double y, double z) {
    Eigen::Matrix3d R;
    R << 1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w,
         2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w,
         2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y;
    return R;
}

std::vector<double> CoordinateTransform::rot_to_quat(const Eigen::Matrix3d& R) {
    double tr = R.trace();
    double w, x, y, z;
    double S;
    
    if (tr > 0) {
        S = std::sqrt(std::max(0.0, tr + 1.0)) * 2;
        w = 0.25 * S;
        x = (R(2,1) - R(1,2)) / S;
        y = (R(0,2) - R(2,0)) / S;
        z = (R(1,0) - R(0,1)) / S;
    } else if ((R(0,0) > R(1,1)) && (R(0,0) > R(2,2))) {
        S = std::sqrt(std::max(0.0, 1.0 + R(0,0) - R(1,1) - R(2,2))) * 2;
        w = (R(2,1) - R(1,2)) / S;
        x = 0.25 * S;
        y = (R(0,1) + R(1,0)) / S;
        z = (R(0,2) + R(2,0)) / S;
    } else if (R(1,1) > R(2,2)) {
        S = std::sqrt(std::max(0.0, 1.0 + R(1,1) - R(0,0) - R(2,2))) * 2;
        w = (R(0,2) - R(2,0)) / S;
        x = (R(0,1) + R(1,0)) / S;
        y = 0.25 * S;
        z = (R(1,2) + R(2,1)) / S;
    } else {
        S = std::sqrt(std::max(0.0, 1.0 + R(2,2) - R(0,0) - R(1,1))) * 2;
        w = (R(1,0) - R(0,1)) / S;
        x = (R(0,2) + R(2,0)) / S;
        y = (R(1,2) + R(2,1)) / S;
        z = 0.25 * S;
    }
    
    double norm = std::sqrt(w*w + x*x + y*y + z*z);
    if (norm > 1e-10) {
        w /= norm;
        x /= norm;
        y /= norm;
        z /= norm;
    }
    
    return {w, x, y, z};
}

std::vector<double> CoordinateTransform::ned_to_enu_position(double x, double y, double z) {
    return {y, x, -z};
}

std::vector<double> CoordinateTransform::enu_to_ned_position(double x, double y, double z) {
    return {y, x, -z};
}

std::vector<double> CoordinateTransform::ned_to_enu_velocity(double vx, double vy, double vz) {
    return {vy, vx, -vz};
}

std::vector<double> CoordinateTransform::enu_to_ned_velocity(double vx, double vy, double vz) {
    return {vy, vx, -vz};
}

std::vector<double> CoordinateTransform::ned_to_enu_acceleration(double ax, double ay, double az) {
    return {ay, ax, -az};
}

std::vector<double> CoordinateTransform::enu_to_ned_acceleration(double ax, double ay, double az) {
    return {ay, ax, -az};
}

std::vector<double> CoordinateTransform::ned_to_enu_quaternion(double qw, double qx, double qy, double qz) {
    Eigen::Matrix3d R_ned = quat_to_rot(qw, qx, qy, qz);
    Eigen::Matrix3d T;
    T << 0, 1, 0,
         1, 0, 0,
         0, 0, -1;
    Eigen::Matrix3d R_enu = T * R_ned;
    return rot_to_quat(R_enu);
}

std::vector<double> CoordinateTransform::enu_to_ned_quaternion(double qw, double qx, double qy, double qz) {
    Eigen::Matrix3d R_enu = quat_to_rot(qw, qx, qy, qz);
    Eigen::Matrix3d T;
    T << 0, 1, 0,
         1, 0, 0,
         0, 0, -1;
    Eigen::Matrix3d R_ned = T.transpose() * R_enu;
    return rot_to_quat(R_ned);
}

std::vector<double> CoordinateTransform::frd_to_flu_position(double x, double y, double z) {
    return {x, -y, -z};
}

std::vector<double> CoordinateTransform::flu_to_frd_position(double x, double y, double z) {
    return {x, -y, -z};
}

std::vector<double> CoordinateTransform::frd_to_flu_velocity(double vx, double vy, double vz) {
    return {vx, -vy, -vz};
}

std::vector<double> CoordinateTransform::flu_to_frd_velocity(double vx, double vy, double vz) {
    return {vx, -vy, -vz};
}

std::vector<double> CoordinateTransform::frd_to_flu_acceleration(double ax, double ay, double az) {
    return {ax, -ay, -az};
}

std::vector<double> CoordinateTransform::flu_to_frd_acceleration(double ax, double ay, double az) {
    return {ax, -ay, -az};
}

std::vector<double> CoordinateTransform::frd_to_flu_quaternion(double qw, double qx, double qy, double qz) {
    Eigen::Matrix3d T;
    T << 1, 0, 0,
         0, -1, 0,
         0, 0, -1;
    Eigen::Matrix3d R_frd = quat_to_rot(qw, qx, qy, qz);
    Eigen::Matrix3d R_flu = T * R_frd * T;
    return rot_to_quat(R_flu);
}

std::vector<double> CoordinateTransform::flu_to_frd_quaternion(double qw, double qx, double qy, double qz) {
    Eigen::Matrix3d T;
    T << 1, 0, 0,
         0, -1, 0,
         0, 0, -1;
    Eigen::Matrix3d R_flu = quat_to_rot(qw, qx, qy, qz);
    Eigen::Matrix3d R_frd = T * R_flu * T;
    return rot_to_quat(R_frd);
}

Eigen::Matrix2d CoordinateTransform::body_to_ned_matrix(double heading) {
    double c = std::cos(heading);
    double s = std::sin(heading);
    Eigen::Matrix2d R;
    R << c, s,
         s, -c;
    return R;
}

Eigen::Matrix2d CoordinateTransform::ned_to_body_matrix(double heading) {
    double c = std::cos(heading);
    double s = std::sin(heading);
    Eigen::Matrix2d R;
    R << c, s,
         s, -c;
    return R;
}

Eigen::Matrix3d CoordinateTransform::body_to_ned_3d_matrix(double heading) {
    double c = std::cos(heading);
    double s = std::sin(heading);
    Eigen::Matrix3d R;
    R << c, s, 0,
         s, -c, 0,
         0, 0, -1;
    return R;
}

Eigen::Matrix3d CoordinateTransform::ned_to_body_3d_matrix(double heading) {
    double c = std::cos(heading);
    double s = std::sin(heading);
    Eigen::Matrix3d R;
    R << c, s, 0,
         s, -c, 0,
         0, 0, -1;
    return R;
}

std::vector<double> CoordinateTransform::flu_to_ned_position(double flu_x, double flu_y, double flu_z, double heading) {
    Eigen::Matrix3d R = body_to_ned_3d_matrix(heading);
    Eigen::Vector3d ned_vec = R * Eigen::Vector3d(flu_x, flu_y, flu_z);
    return {ned_vec(0), ned_vec(1), ned_vec(2)};
}

std::vector<double> CoordinateTransform::ned_to_flu_position(double ned_x, double ned_y, double ned_z, double heading) {
    Eigen::Matrix3d R = ned_to_body_3d_matrix(heading);
    Eigen::Vector3d flu_vec = R * Eigen::Vector3d(ned_x, ned_y, ned_z);
    return {flu_vec(0), flu_vec(1), flu_vec(2)};
}

std::vector<double> CoordinateTransform::flu_to_ned_velocity(double flu_vx, double flu_vy, double flu_vz, double heading) {
    Eigen::Matrix3d R = body_to_ned_3d_matrix(heading);
    Eigen::Vector3d ned_vec = R * Eigen::Vector3d(flu_vx, flu_vy, flu_vz);
    return {ned_vec(0), ned_vec(1), ned_vec(2)};
}

std::vector<double> CoordinateTransform::ned_to_flu_velocity(double ned_vx, double ned_vy, double ned_vz, double heading) {
    Eigen::Matrix3d R = ned_to_body_3d_matrix(heading);
    Eigen::Vector3d flu_vec = R * Eigen::Vector3d(ned_vx, ned_vy, ned_vz);
    return {flu_vec(0), flu_vec(1), flu_vec(2)};
}

std::vector<double> CoordinateTransform::flu_to_ned_acceleration(double flu_ax, double flu_ay, double flu_az, double heading) {
    Eigen::Matrix3d R = body_to_ned_3d_matrix(heading);
    Eigen::Vector3d ned_vec = R * Eigen::Vector3d(flu_ax, flu_ay, flu_az);
    return {ned_vec(0), ned_vec(1), ned_vec(2)};
}

std::vector<double> CoordinateTransform::ned_to_flu_acceleration(double ned_ax, double ned_ay, double ned_az, double heading) {
    Eigen::Matrix3d R = ned_to_body_3d_matrix(heading);
    Eigen::Vector3d flu_vec = R * Eigen::Vector3d(ned_ax, ned_ay, ned_az);
    return {flu_vec(0), flu_vec(1), flu_vec(2)};
}

std::vector<double> CoordinateTransform::flu_to_frd_angular_velocity(double flu_wx, double flu_wy, double flu_wz) {
    return {flu_wx, -flu_wy, -flu_wz};
}

std::vector<double> CoordinateTransform::frd_to_flu_angular_velocity(double frd_wx, double frd_wy, double frd_wz) {
    return {frd_wx, -frd_wy, -frd_wz};
}

std::vector<double> CoordinateTransform::frd_to_ned_velocity(double vx, double vy, double vz, double heading) {
    double flu_vx = vx;
    double flu_vy = -vy;
    double flu_vz = -vz;
    return flu_to_ned_velocity(flu_vx, flu_vy, flu_vz, heading);
}

std::vector<double> CoordinateTransform::ned_to_frd_velocity(double vx, double vy, double vz, double heading) {
    std::vector<double> flu_vel = ned_to_flu_velocity(vx, vy, vz, heading);
    return {flu_vel[0], -flu_vel[1], -flu_vel[2]};
}

std::vector<double> CoordinateTransform::frd_ned_to_flu_enu_velocity(double frd_vx, double frd_vy, double frd_vz, double heading) {
    double flu_vx = frd_vx;
    double flu_vy = -frd_vy;
    double flu_vz = -frd_vz;
    std::vector<double> ned_vel = flu_to_ned_velocity(flu_vx, flu_vy, flu_vz, heading);
    return ned_to_enu_velocity(ned_vel[0], ned_vel[1], ned_vel[2]);
}

std::vector<double> CoordinateTransform::flu_enu_to_frd_ned_velocity(double flu_vx, double flu_vy, double flu_vz, double heading) {
    std::vector<double> ned_vel = enu_to_ned_velocity(flu_vx, flu_vy, flu_vz);
    std::vector<double> flu_vel = ned_to_flu_velocity(ned_vel[0], ned_vel[1], ned_vel[2], heading);
    return {flu_vel[0], -flu_vel[1], -flu_vel[2]};
}

std::vector<double> CoordinateTransform::frd_ned_to_flu_enu_quaternion(double frd_qw, double frd_qx, double frd_qy, double frd_qz) {
    Eigen::Matrix3d R_ned_to_enu;
    R_ned_to_enu << 0, 1, 0,
                    1, 0, 0,
                    0, 0, -1;
    Eigen::Matrix3d R_frd_to_flu;
    R_frd_to_flu << 1, 0, 0,
                    0, -1, 0,
                    0, 0, -1;
    
    Eigen::Matrix3d R_frd_ned = quat_to_rot(frd_qw, frd_qx, frd_qy, frd_qz);
    Eigen::Matrix3d R_flu_enu = R_ned_to_enu * R_frd_ned * R_frd_to_flu;
    
    return rot_to_quat(R_flu_enu);
}

std::vector<double> CoordinateTransform::flu_enu_to_frd_ned_quaternion(double flu_qw, double flu_qx, double flu_qy, double flu_qz) {
    Eigen::Matrix3d R_enu_to_ned;
    R_enu_to_ned << 0, 1, 0,
                    1, 0, 0,
                    0, 0, -1;
    Eigen::Matrix3d R_flu_to_frd;
    R_flu_to_frd << 1, 0, 0,
                    0, -1, 0,
                    0, 0, -1;
    
    Eigen::Matrix3d R_flu_enu = quat_to_rot(flu_qw, flu_qx, flu_qy, flu_qz);
    Eigen::Matrix3d R_frd_ned = R_enu_to_ned * R_flu_enu * R_flu_to_frd;
    
    return rot_to_quat(R_frd_ned);
}

double CoordinateTransform::flu_to_ned_yawspeed(double flu_wz) {
    return -flu_wz;
}

double CoordinateTransform::ned_to_flu_yawspeed(double ned_wz) {
    return -ned_wz;
}

std::vector<double> CoordinateTransform::flu_to_enu_position(double flu_x, double flu_y, double flu_z, double heading) {
    std::vector<double> ned_pos = flu_to_ned_position(flu_x, flu_y, flu_z, heading);
    return ned_to_enu_position(ned_pos[0], ned_pos[1], ned_pos[2]);
}

std::vector<double> CoordinateTransform::enu_to_flu_position(double enu_x, double enu_y, double enu_z, double heading) {
    std::vector<double> ned_pos = enu_to_ned_position(enu_x, enu_y, enu_z);
    return ned_to_flu_position(ned_pos[0], ned_pos[1], ned_pos[2], heading);
}

std::vector<double> CoordinateTransform::flu_to_enu_velocity(double flu_vx, double flu_vy, double flu_vz, double heading) {
    std::vector<double> ned_vel = flu_to_ned_velocity(flu_vx, flu_vy, flu_vz, heading);
    return ned_to_enu_velocity(ned_vel[0], ned_vel[1], ned_vel[2]);
}

std::vector<double> CoordinateTransform::enu_to_flu_velocity(double enu_vx, double enu_vy, double enu_vz, double heading) {
    std::vector<double> ned_vel = enu_to_ned_velocity(enu_vx, enu_vy, enu_vz);
    return ned_to_flu_velocity(ned_vel[0], ned_vel[1], ned_vel[2], heading);
}

std::vector<double> CoordinateTransform::flu_to_enu_acceleration(double flu_ax, double flu_ay, double flu_az, double heading) {
    std::vector<double> ned_accel = flu_to_ned_acceleration(flu_ax, flu_ay, flu_az, heading);
    return ned_to_enu_acceleration(ned_accel[0], ned_accel[1], ned_accel[2]);
}

std::vector<double> CoordinateTransform::enu_to_flu_acceleration(double enu_ax, double enu_ay, double enu_az, double heading) {
    std::vector<double> ned_accel = enu_to_ned_acceleration(enu_ax, enu_ay, enu_az);
    return ned_to_flu_acceleration(ned_accel[0], ned_accel[1], ned_accel[2], heading);
}

Eigen::Vector3d CoordinateTransform::flu_to_enu_vector_by_rotation_matrix(const Eigen::Vector3d& flu_vec, const Eigen::Matrix3d& rotation_matrix) {
    return rotation_matrix * flu_vec;
}

Eigen::Vector3d CoordinateTransform::enu_to_flu_vector_by_rotation_matrix(const Eigen::Vector3d& enu_vec, const Eigen::Matrix3d& rotation_matrix) {
    return rotation_matrix.transpose() * enu_vec;
}

Eigen::Vector3d CoordinateTransform::flu_to_ned_vector_by_rotation_matrix(const Eigen::Vector3d& flu_vec, const Eigen::Matrix3d& rotation_matrix) {
    Eigen::Vector3d enu_vec = rotation_matrix * flu_vec;
    std::vector<double> ned_vec = enu_to_ned_velocity(enu_vec(0), enu_vec(1), enu_vec(2));
    return Eigen::Vector3d(ned_vec[0], ned_vec[1], ned_vec[2]);
}

std::tuple<double, double, double> CoordinateTransform::flu_to_ned_position_px4_odom(
    double flu_x, double flu_y, double flu_z,
    double init_n, double init_e, double init_d, double init_heading) {
    
    std::vector<double> ned_offset = flu_to_ned_position(flu_x, flu_y, flu_z, init_heading);
    
    double compensated_n = init_n + ned_offset[0];
    double compensated_e = init_e + ned_offset[1];
    double compensated_d = init_d + ned_offset[2];
    
    return std::make_tuple(compensated_n, compensated_e, compensated_d);
}

std::vector<double> CoordinateTransform::flu_to_ned_quaternion_px4_odom(
    double flu_qw, double flu_qx, double flu_qy, double flu_qz, double init_heading) {
    return flu_enu_to_frd_ned_quaternion(flu_qw, flu_qx, flu_qy, flu_qz);
}

std::vector<double> CoordinateTransform::qmult(
    double q1_w, double q1_x, double q1_y, double q1_z,
    double q2_w, double q2_x, double q2_y, double q2_z) {
    
    double w = q1_w * q2_w - q1_x * q2_x - q1_y * q2_y - q1_z * q2_z;
    double x = q1_w * q2_x + q1_x * q2_w + q1_y * q2_z - q1_z * q2_y;
    double y = q1_w * q2_y - q1_x * q2_z + q1_y * q2_w + q1_z * q2_x;
    double z = q1_w * q2_z + q1_x * q2_y - q1_y * q2_x + q1_z * q2_w;
    
    return {w, x, y, z};
}

Eigen::Matrix3d CoordinateTransform::quat2mat(double w, double x, double y, double z) {
    return quat_to_rot(w, x, y, z);
}

std::tuple<std::vector<double>, std::vector<double>> CoordinateTransform::inverse_transform(
    double pos_x, double pos_y, double pos_z,
    double quat_w, double quat_x, double quat_y, double quat_z) {
    
    Eigen::Matrix3d R = quat_to_rot(quat_w, quat_x, quat_y, quat_z);
    Eigen::Vector3d pos(pos_x, pos_y, pos_z);
    
    Eigen::Vector3d inv_pos = -R.transpose() * pos;
    
    std::vector<double> inv_quat = {quat_w, -quat_x, -quat_y, -quat_z};
    
    double norm = std::sqrt(inv_quat[0]*inv_quat[0] + inv_quat[1]*inv_quat[1] + 
                           inv_quat[2]*inv_quat[2] + inv_quat[3]*inv_quat[3]);
    if (norm > 1e-10) {
        inv_quat[0] /= norm;
        inv_quat[1] /= norm;
        inv_quat[2] /= norm;
        inv_quat[3] /= norm;
    }
    
    return std::make_tuple(
        std::vector<double>{inv_pos(0), inv_pos(1), inv_pos(2)},
        inv_quat
    );
}

double CoordinateTransform::heading_from_quaternion(double w, double x, double y, double z) {
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    return std::atan2(siny_cosp, cosy_cosp);
}

Eigen::Matrix3d CoordinateTransform::create_rotation_matrix_from_quaternion(
    double w, double x, double y, double z) {
    return quat_to_rot(w, x, y, z);
}

std::tuple<std::vector<double>, std::vector<double>> CoordinateTransform::multiply_transforms(
    double t1_x, double t1_y, double t1_z, double t1_qw, double t1_qx, double t1_qy, double t1_qz,
    double t2_x, double t2_y, double t2_z, double t2_qw, double t2_qx, double t2_qy, double t2_qz) {
    
    std::vector<double> result_quat = qmult(t1_qw, t1_qx, t1_qy, t1_qz, t2_qw, t2_qx, t2_qy, t2_qz);
    
    Eigen::Matrix3d R1 = quat_to_rot(t1_qw, t1_qx, t1_qy, t1_qz);
    Eigen::Vector3d t1_trans(t1_x, t1_y, t1_z);
    Eigen::Vector3d t2_trans(t2_x, t2_y, t2_z);
    
    Eigen::Vector3d result_trans = t1_trans + R1 * t2_trans;
    
    return std::make_tuple(
        std::vector<double>{result_trans(0), result_trans(1), result_trans(2)},
        result_quat
    );
}

}
