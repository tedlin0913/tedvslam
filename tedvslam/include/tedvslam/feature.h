#pragma once

#ifndef FEATURE_H
#define FEATURE_H

#include <memory>
#include <opencv2/features2d.hpp>
#include "tedvslam/common_include.h"

namespace tedvslam
{

    class Frame;
    class MapPoint;
    class KeyFrame;

    class Feature
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        using Ptr = std::shared_ptr<Feature>;

        Feature() = default;

        explicit Feature(const cv::KeyPoint &keypoint);

        Feature(const std::shared_ptr<KeyFrame> &keyframe, const cv::KeyPoint &keypoint);

        static Ptr Create(const cv::KeyPoint &keypoint);

        static Ptr Create(const std::shared_ptr<KeyFrame> &keyframe, const cv::KeyPoint &keypoint);

    public:
        std::weak_ptr<KeyFrame> keyframe_;
        cv::KeyPoint keypoint_;                       // Keypoint position
        std::vector<cv::KeyPoint> pyramid_keypoints_; // Pyramid keypoints
        std::weak_ptr<MapPoint> map_point_;           // Associated map point

        bool is_on_left_frame_ = true; // True: on left frame; False: on right frame
        bool is_outlier_ = false;      // Outlier flag
    };
} // namespace myslam

#endif // FEATURE_H
