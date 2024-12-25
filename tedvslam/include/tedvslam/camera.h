#pragma once

#ifndef CAMERA_H
#define CAMERA_H

#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>

#include "tedvslam/common_include.h"

namespace tedvslam
{

    // Pinhole stereo camera model
    class Camera
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        using Ptr = std::shared_ptr<Camera>;

        double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0, baseline_ = 0; // Camera intrinsics
        SE3 pose_;                                                // Extrinsic: from stereo camera to single camera
        SE3 pose_inv_;                                            // Inverse of extrinsic

        Camera() = default;

        Camera(double fx, double fy, double cx, double cy, double baseline, const SE3 &pose)
            : fx_(fx), fy_(fy), cx_(cx), cy_(cy), baseline_(baseline), pose_(pose), pose_inv_(pose.inverse()) {}

        SE3 pose() const { return pose_; }

        // Return intrinsic matrix
        Eigen::Matrix3d K() const
        {
            Eigen::Matrix3d k;
            k << fx_, 0, cx_,
                0, fy_, cy_,
                0, 0, 1;
            return k;
        }

        // Coordinate transforms: world, camera, pixel
        Eigen::Vector3d world2camera(const Eigen::Vector3d &p_w, const SE3 &T_c_w) const;
        Eigen::Vector3d camera2world(const Eigen::Vector3d &p_c, const SE3 &T_c_w) const;
        Eigen::Vector2d camera2pixel(const Eigen::Vector3d &p_c) const;
        Eigen::Vector3d pixel2camera(const Eigen::Vector2d &p_p, double depth = 1) const;
        Eigen::Vector3d pixel2world(const Eigen::Vector2d &p_p, const SE3 &T_c_w, double depth = 1) const;
        Eigen::Vector2d world2pixel(const Eigen::Vector3d &p_w, const SE3 &T_c_w) const;
        Eigen::Vector3d pixel2cameraIdentity(const Eigen::Vector2d &p_p, double depth = 1) const;
        // Undistort the image
        void undistortImage(const cv::Mat &src, cv::Mat &dst, const cv::Mat &distCoeffs) const;
    };

} // namespace tedvslam
#endif // CAMERA_H
