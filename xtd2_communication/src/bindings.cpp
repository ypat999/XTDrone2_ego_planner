#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <pybind11/eigen.h>
#include "xtd2_communication/coordinate_transform.hpp"

namespace py = pybind11;
using namespace xtd2_communication;

PYBIND11_MODULE(coordinate_transform_cpp, m) {
    m.doc() = "Coordinate transformation utilities implemented in C++";

    py::class_<CoordinateTransform>(m, "CoordinateTransform")
        .def(py::init<>())
        
        .def_static("ned_to_enu_position", &CoordinateTransform::ned_to_enu_position,
            "NED position -> ENU position",
            py::arg("x"), py::arg("y"), py::arg("z"))
        
        .def_static("enu_to_ned_position", &CoordinateTransform::enu_to_ned_position,
            "ENU position -> NED position",
            py::arg("x"), py::arg("y"), py::arg("z"))
        
        .def_static("ned_to_enu_velocity", &CoordinateTransform::ned_to_enu_velocity,
            "NED velocity -> ENU velocity",
            py::arg("vx"), py::arg("vy"), py::arg("vz"))
        
        .def_static("enu_to_ned_velocity", &CoordinateTransform::enu_to_ned_velocity,
            "ENU velocity -> NED velocity",
            py::arg("vx"), py::arg("vy"), py::arg("vz"))
        
        .def_static("ned_to_enu_acceleration", &CoordinateTransform::ned_to_enu_acceleration,
            "NED acceleration -> ENU acceleration",
            py::arg("ax"), py::arg("ay"), py::arg("az"))
        
        .def_static("enu_to_ned_acceleration", &CoordinateTransform::enu_to_ned_acceleration,
            "ENU acceleration -> NED acceleration",
            py::arg("ax"), py::arg("ay"), py::arg("az"))
        
        .def_static("ned_to_enu_quaternion", &CoordinateTransform::ned_to_enu_quaternion,
            "NED quaternion -> ENU quaternion",
            py::arg("qw"), py::arg("qx"), py::arg("qy"), py::arg("qz"))
        
        .def_static("enu_to_ned_quaternion", &CoordinateTransform::enu_to_ned_quaternion,
            "ENU quaternion -> NED quaternion",
            py::arg("qw"), py::arg("qx"), py::arg("qy"), py::arg("qz"))
        
        .def_static("frd_to_flu_position", &CoordinateTransform::frd_to_flu_position,
            "FRD position -> FLU position",
            py::arg("x"), py::arg("y"), py::arg("z"))
        
        .def_static("flu_to_frd_position", &CoordinateTransform::flu_to_frd_position,
            "FLU position -> FRD position",
            py::arg("x"), py::arg("y"), py::arg("z"))
        
        .def_static("frd_to_flu_velocity", &CoordinateTransform::frd_to_flu_velocity,
            "FRD velocity -> FLU velocity",
            py::arg("vx"), py::arg("vy"), py::arg("vz"))
        
        .def_static("flu_to_frd_velocity", &CoordinateTransform::flu_to_frd_velocity,
            "FLU velocity -> FRD velocity",
            py::arg("vx"), py::arg("vy"), py::arg("vz"))
        
        .def_static("frd_to_flu_acceleration", &CoordinateTransform::frd_to_flu_acceleration,
            "FRD acceleration -> FLU acceleration",
            py::arg("ax"), py::arg("ay"), py::arg("az"))
        
        .def_static("flu_to_frd_acceleration", &CoordinateTransform::flu_to_frd_acceleration,
            "FLU acceleration -> FRD acceleration",
            py::arg("ax"), py::arg("ay"), py::arg("az"))
        
        .def_static("frd_to_flu_quaternion", &CoordinateTransform::frd_to_flu_quaternion,
            "FRD quaternion -> FLU quaternion",
            py::arg("qw"), py::arg("qx"), py::arg("qy"), py::arg("qz"))
        
        .def_static("flu_to_frd_quaternion", &CoordinateTransform::flu_to_frd_quaternion,
            "FLU quaternion -> FRD quaternion",
            py::arg("qw"), py::arg("qx"), py::arg("qy"), py::arg("qz"))
        
        .def_static("body_to_ned_matrix", [](double heading) {
            return CoordinateTransform::body_to_ned_matrix(heading);
        }, "Create 2D reflection matrix from FLU to NED",
           py::arg("heading"))
        
        .def_static("ned_to_body_matrix", [](double heading) {
            return CoordinateTransform::ned_to_body_matrix(heading);
        }, "Create 2D reflection matrix from NED to FLU",
           py::arg("heading"))
        
        .def_static("body_to_ned_3d_matrix", [](double heading) {
            return CoordinateTransform::body_to_ned_3d_matrix(heading);
        }, "Create 3D transformation matrix from FLU to NED",
           py::arg("heading"))
        
        .def_static("ned_to_body_3d_matrix", [](double heading) {
            return CoordinateTransform::ned_to_body_3d_matrix(heading);
        }, "Create 3D transformation matrix from NED to FLU",
           py::arg("heading"))
        
        .def_static("flu_to_ned_position", &CoordinateTransform::flu_to_ned_position,
            "FLU position -> NED position",
            py::arg("flu_x"), py::arg("flu_y"), py::arg("flu_z"), py::arg("heading"))
        
        .def_static("ned_to_flu_position", &CoordinateTransform::ned_to_flu_position,
            "NED position -> FLU position",
            py::arg("ned_x"), py::arg("ned_y"), py::arg("ned_z"), py::arg("heading"))
        
        .def_static("flu_to_ned_velocity", &CoordinateTransform::flu_to_ned_velocity,
            "FLU velocity -> NED velocity",
            py::arg("flu_vx"), py::arg("flu_vy"), py::arg("flu_vz"), py::arg("heading"))
        
        .def_static("ned_to_flu_velocity", &CoordinateTransform::ned_to_flu_velocity,
            "NED velocity -> FLU velocity",
            py::arg("ned_vx"), py::arg("ned_vy"), py::arg("ned_vz"), py::arg("heading"))
        
        .def_static("flu_to_ned_acceleration", &CoordinateTransform::flu_to_ned_acceleration,
            "FLU acceleration -> NED acceleration",
            py::arg("flu_ax"), py::arg("flu_ay"), py::arg("flu_az"), py::arg("heading"))
        
        .def_static("ned_to_flu_acceleration", &CoordinateTransform::ned_to_flu_acceleration,
            "NED acceleration -> FLU acceleration",
            py::arg("ned_ax"), py::arg("ned_ay"), py::arg("ned_az"), py::arg("heading"))
        
        .def_static("flu_to_frd_angular_velocity", &CoordinateTransform::flu_to_frd_angular_velocity,
            "FLU angular velocity -> FRD angular velocity",
            py::arg("flu_wx"), py::arg("flu_wy"), py::arg("flu_wz"))
        
        .def_static("frd_to_flu_angular_velocity", &CoordinateTransform::frd_to_flu_angular_velocity,
            "FRD angular velocity -> FLU angular velocity",
            py::arg("frd_wx"), py::arg("frd_wy"), py::arg("frd_wz"))
        
        .def_static("frd_to_ned_velocity", &CoordinateTransform::frd_to_ned_velocity,
            "FRD velocity -> NED velocity",
            py::arg("vx"), py::arg("vy"), py::arg("vz"), py::arg("heading"))
        
        .def_static("ned_to_frd_velocity", &CoordinateTransform::ned_to_frd_velocity,
            "NED velocity -> FRD velocity",
            py::arg("vx"), py::arg("vy"), py::arg("vz"), py::arg("heading"))
        
        .def_static("frd_ned_to_flu_enu_velocity", &CoordinateTransform::frd_ned_to_flu_enu_velocity,
            "vehicle_odometry velocity: FRD -> FLU -> ENU",
            py::arg("frd_vx"), py::arg("frd_vy"), py::arg("frd_vz"), py::arg("heading"))
        
        .def_static("flu_enu_to_frd_ned_velocity", &CoordinateTransform::flu_enu_to_frd_ned_velocity,
            "velocity: ENU -> NED -> FLU -> FRD",
            py::arg("flu_vx"), py::arg("flu_vy"), py::arg("flu_vz"), py::arg("heading"))
        
        .def_static("frd_ned_to_flu_enu_quaternion", &CoordinateTransform::frd_ned_to_flu_enu_quaternion,
            "vehicle_odometry quaternion: FRD/NED -> FLU/ENU",
            py::arg("frd_qw"), py::arg("frd_qx"), py::arg("frd_qy"), py::arg("frd_qz"))
        
        .def_static("flu_enu_to_frd_ned_quaternion", &CoordinateTransform::flu_enu_to_frd_ned_quaternion,
            "quaternion: FLU/ENU -> FRD/NED",
            py::arg("flu_qw"), py::arg("flu_qx"), py::arg("flu_qy"), py::arg("flu_qz"))
        
        .def_static("flu_to_ned_yawspeed", &CoordinateTransform::flu_to_ned_yawspeed,
            "FLU yawspeed -> NED yawspeed",
            py::arg("flu_wz"))
        
        .def_static("ned_to_flu_yawspeed", &CoordinateTransform::ned_to_flu_yawspeed,
            "NED yawspeed -> FLU yawspeed",
            py::arg("ned_wz"))
        
        .def_static("flu_to_enu_position", &CoordinateTransform::flu_to_enu_position,
            "FLU position -> ENU position",
            py::arg("flu_x"), py::arg("flu_y"), py::arg("flu_z"), py::arg("heading"))
        
        .def_static("enu_to_flu_position", &CoordinateTransform::enu_to_flu_position,
            "ENU position -> FLU position",
            py::arg("enu_x"), py::arg("enu_y"), py::arg("enu_z"), py::arg("heading"))
        
        .def_static("flu_to_enu_velocity", &CoordinateTransform::flu_to_enu_velocity,
            "FLU velocity -> ENU velocity",
            py::arg("flu_vx"), py::arg("flu_vy"), py::arg("flu_vz"), py::arg("heading"))
        
        .def_static("enu_to_flu_velocity", &CoordinateTransform::enu_to_flu_velocity,
            "ENU velocity -> FLU velocity",
            py::arg("enu_vx"), py::arg("enu_vy"), py::arg("enu_vz"), py::arg("heading"))
        
        .def_static("flu_to_enu_acceleration", &CoordinateTransform::flu_to_enu_acceleration,
            "FLU acceleration -> ENU acceleration",
            py::arg("flu_ax"), py::arg("flu_ay"), py::arg("flu_az"), py::arg("heading"))
        
        .def_static("enu_to_flu_acceleration", &CoordinateTransform::enu_to_flu_acceleration,
            "ENU acceleration -> FLU acceleration",
            py::arg("enu_ax"), py::arg("enu_ay"), py::arg("enu_az"), py::arg("heading"))
        
        .def_static("flu_to_enu_vector_by_rotation_matrix", [](py::array_t<double> flu_vec, py::array_t<double> rotation_matrix) {
            py::buffer_info flu_info = flu_vec.request();
            py::buffer_info rot_info = rotation_matrix.request();
            
            if (flu_info.ndim != 1 || flu_info.shape[0] != 3) {
                throw std::runtime_error("flu_vec must be a 1D array of size 3");
            }
            if (rot_info.ndim != 2 || rot_info.shape[0] != 3 || rot_info.shape[1] != 3) {
                throw std::runtime_error("rotation_matrix must be a 3x3 array");
            }
            
            Eigen::Map<Eigen::Vector3d> flu_eigen(static_cast<double*>(flu_info.ptr));
            Eigen::Map<Eigen::Matrix3d> rot_eigen(static_cast<double*>(rot_info.ptr));
            
            Eigen::Vector3d result = CoordinateTransform::flu_to_enu_vector_by_rotation_matrix(flu_eigen, rot_eigen);
            
            py::array_t<double> result_array(3);
            py::buffer_info result_info = result_array.request();
            double* result_ptr = static_cast<double*>(result_info.ptr);
            result_ptr[0] = result(0);
            result_ptr[1] = result(1);
            result_ptr[2] = result(2);
            
            return result_array;
        }, "Convert FLU vector to ENU using rotation matrix",
           py::arg("flu_vec"), py::arg("rotation_matrix"))
        
        .def_static("enu_to_flu_vector_by_rotation_matrix", [](py::array_t<double> enu_vec, py::array_t<double> rotation_matrix) {
            py::buffer_info enu_info = enu_vec.request();
            py::buffer_info rot_info = rotation_matrix.request();
            
            if (enu_info.ndim != 1 || enu_info.shape[0] != 3) {
                throw std::runtime_error("enu_vec must be a 1D array of size 3");
            }
            if (rot_info.ndim != 2 || rot_info.shape[0] != 3 || rot_info.shape[1] != 3) {
                throw std::runtime_error("rotation_matrix must be a 3x3 array");
            }
            
            Eigen::Map<Eigen::Vector3d> enu_eigen(static_cast<double*>(enu_info.ptr));
            Eigen::Map<Eigen::Matrix3d> rot_eigen(static_cast<double*>(rot_info.ptr));
            
            Eigen::Vector3d result = CoordinateTransform::enu_to_flu_vector_by_rotation_matrix(enu_eigen, rot_eigen);
            
            py::array_t<double> result_array(3);
            py::buffer_info result_info = result_array.request();
            double* result_ptr = static_cast<double*>(result_info.ptr);
            result_ptr[0] = result(0);
            result_ptr[1] = result(1);
            result_ptr[2] = result(2);
            
            return result_array;
        }, "Convert ENU vector to FLU using rotation matrix",
           py::arg("enu_vec"), py::arg("rotation_matrix"))
        
        .def_static("flu_to_ned_vector_by_rotation_matrix", [](py::array_t<double> flu_vec, py::array_t<double> rotation_matrix) {
            py::buffer_info flu_info = flu_vec.request();
            py::buffer_info rot_info = rotation_matrix.request();
            
            if (flu_info.ndim != 1 || flu_info.shape[0] != 3) {
                throw std::runtime_error("flu_vec must be a 1D array of size 3");
            }
            if (rot_info.ndim != 2 || rot_info.shape[0] != 3 || rot_info.shape[1] != 3) {
                throw std::runtime_error("rotation_matrix must be a 3x3 array");
            }
            
            Eigen::Map<Eigen::Vector3d> flu_eigen(static_cast<double*>(flu_info.ptr));
            Eigen::Map<Eigen::Matrix3d> rot_eigen(static_cast<double*>(rot_info.ptr));
            
            Eigen::Vector3d result = CoordinateTransform::flu_to_ned_vector_by_rotation_matrix(flu_eigen, rot_eigen);
            
            py::array_t<double> result_array(3);
            py::buffer_info result_info = result_array.request();
            double* result_ptr = static_cast<double*>(result_info.ptr);
            result_ptr[0] = result(0);
            result_ptr[1] = result(1);
            result_ptr[2] = result(2);
            
            return result_array;
        }, "Convert FLU vector to NED using rotation matrix",
           py::arg("flu_vec"), py::arg("rotation_matrix"))
        
        .def_static("flu_to_ned_position_px4_odom", [](double flu_x, double flu_y, double flu_z,
                                                        double init_n, double init_e, double init_d, double init_heading) {
            auto result = CoordinateTransform::flu_to_ned_position_px4_odom(flu_x, flu_y, flu_z, init_n, init_e, init_d, init_heading);
            return py::make_tuple(std::get<0>(result), std::get<1>(result), std::get<2>(result));
        }, "px4_odom specific FLU -> NED position conversion",
           py::arg("flu_x"), py::arg("flu_y"), py::arg("flu_z"),
           py::arg("init_n"), py::arg("init_e"), py::arg("init_d"), py::arg("init_heading"))
        
        .def_static("flu_to_ned_quaternion_px4_odom", &CoordinateTransform::flu_to_ned_quaternion_px4_odom,
            "px4_odom specific FLU -> NED quaternion conversion",
            py::arg("flu_qw"), py::arg("flu_qx"), py::arg("flu_qy"), py::arg("flu_qz"), py::arg("init_heading"))
        
        .def_static("qmult", &CoordinateTransform::qmult,
            "Quaternion multiplication: q1 * q2",
            py::arg("q1_w"), py::arg("q1_x"), py::arg("q1_y"), py::arg("q1_z"),
            py::arg("q2_w"), py::arg("q2_x"), py::arg("q2_y"), py::arg("q2_z"))
        
        .def_static("quat2mat", [](double w, double x, double y, double z) {
            return CoordinateTransform::quat2mat(w, x, y, z);
        }, "Convert quaternion to rotation matrix",
           py::arg("w"), py::arg("x"), py::arg("y"), py::arg("z"))
        
        .def_static("inverse_transform", [](double pos_x, double pos_y, double pos_z,
                                             double quat_w, double quat_x, double quat_y, double quat_z) {
            auto result = CoordinateTransform::inverse_transform(pos_x, pos_y, pos_z, quat_w, quat_x, quat_y, quat_z);
            return py::make_tuple(std::get<0>(result), std::get<1>(result));
        }, "Compute inverse transform (position and quaternion)",
           py::arg("pos_x"), py::arg("pos_y"), py::arg("pos_z"),
           py::arg("quat_w"), py::arg("quat_x"), py::arg("quat_y"), py::arg("quat_z"))
        
        .def_static("heading_from_quaternion", &CoordinateTransform::heading_from_quaternion,
            "Extract heading (yaw) from quaternion",
            py::arg("w"), py::arg("x"), py::arg("y"), py::arg("z"))
        
        .def_static("create_rotation_matrix_from_quaternion", [](double w, double x, double y, double z) {
            return CoordinateTransform::create_rotation_matrix_from_quaternion(w, x, y, z);
        }, "Create rotation matrix from quaternion",
           py::arg("w"), py::arg("x"), py::arg("y"), py::arg("z"))
        
        .def_static("multiply_transforms", [](double t1_x, double t1_y, double t1_z,
                                               double t1_qw, double t1_qx, double t1_qy, double t1_qz,
                                               double t2_x, double t2_y, double t2_z,
                                               double t2_qw, double t2_qx, double t2_qy, double t2_qz) {
            auto result = CoordinateTransform::multiply_transforms(
                t1_x, t1_y, t1_z, t1_qw, t1_qx, t1_qy, t1_qz,
                t2_x, t2_y, t2_z, t2_qw, t2_qx, t2_qy, t2_qz);
            return py::make_tuple(std::get<0>(result), std::get<1>(result));
        }, "Multiply two transforms: result = t1 * t2",
           py::arg("t1_x"), py::arg("t1_y"), py::arg("t1_z"),
           py::arg("t1_qw"), py::arg("t1_qx"), py::arg("t1_qy"), py::arg("t1_qz"),
           py::arg("t2_x"), py::arg("t2_y"), py::arg("t2_z"),
           py::arg("t2_qw"), py::arg("t2_qx"), py::arg("t2_qy"), py::arg("t2_qz"));
}
