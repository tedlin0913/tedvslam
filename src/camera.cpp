#include "tedvslam/camera.h"
#include <opencv2/calib3d.hpp>

namespace tedvslam
{

    Eigen::Vector3d Camera::world2camera(const Eigen::Vector3d &p_w, const SE3 &T_c_w) const
    {
        return pose_ * T_c_w * p_w;
    }

    Eigen::Vector3d Camera::camera2world(const Eigen::Vector3d &p_c, const SE3 &T_c_w) const
    {
        return T_c_w.inverse() * pose_inv_ * p_c;
    }

    Eigen::Vector2d Camera::camera2pixel(const Eigen::Vector3d &p_c) const
    {
        return Eigen::Vector2d(
            fx_ * p_c.x() / p_c.z() + cx_,
            fy_ * p_c.y() / p_c.z() + cy_);
    }

    Eigen::Vector3d Camera::pixel2camera(const Eigen::Vector2d &p_p, double depth) const
    {
        return Eigen::Vector3d(
            (p_p.x() - cx_) * depth / fx_,
            (p_p.y() - cy_) * depth / fy_,
            depth);
    }

    Eigen::Vector3d Camera::pixel2cameraIdentity(const Eigen::Vector2d &p_p, double depth) const
    {
        return Eigen::Vector3d(
            (p_p.x()) * depth,
            (p_p.y()) * depth,
            depth);
    }

    Eigen::Vector3d Camera::pixel2world(const Eigen::Vector2d &p_p, const SE3 &T_c_w, double depth) const
    {
        return camera2world(pixel2camera(p_p, depth), T_c_w);
    }

    Eigen::Vector2d Camera::world2pixel(const Eigen::Vector3d &p_w, const SE3 &T_c_w) const
    {
        return camera2pixel(world2camera(p_w, T_c_w));
    }

    void Camera::undistortImage(const cv::Mat &src, cv::Mat &dst, const cv::Mat &distCoeffs) const
    {
        cv::Mat K_cv = (cv::Mat_<float>(3, 3) << fx_, 0, cx_,
                        0, fy_, cy_,
                        0, 0, 1);
        cv::Mat temp_dst;
        cv::undistort(src, temp_dst, K_cv, distCoeffs);
        dst = temp_dst.clone(); // Assign to `dst` safely
    }

}
