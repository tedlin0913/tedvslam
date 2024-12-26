#pragma once

#ifndef FRAME_H
#define FRAME_H

#include "tedvslam/camera.h"
#include "tedvslam/common_include.h"

namespace tedvslam
{

    // Forward declaration
    class Feature;

    class Frame
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        using Ptr = std::shared_ptr<Frame>;
        Frame(const cv::Mat &left_img, const cv::Mat &right_img, double timestamp);

        Frame() = default;
        static Ptr Create();
        static Ptr Create(const cv::Mat &left_img, const cv::Mat &right_img, double timestamp);

        void SetPose(const SE3 &pose);

        // Sets the relative pose to the reference keyframe.
        void SetRelativePose(const SE3 &relative_pose);

        SE3 Pose() const;

        SE3 RelativePose() const;

    public:
        unsigned long frame_id_;
        double timestamp_;

        cv::Mat left_img_, right_img_;

        std::vector<std::shared_ptr<Feature>> features_left_;
        std::vector<std::shared_ptr<Feature>> features_right_;

    private:
        SE3 pose_;          // Used for the viewer.
        SE3 relative_pose_; // Used for tracking.

        mutable std::mutex pose_mutex_;
        mutable std::mutex relative_pose_mutex_;
    };

} // namespace tedvslam

#endif
