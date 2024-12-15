#pragma once

#ifndef FEATURE_H
#define FEATURE_H

#include <memory>
#include <opencv2/features2d.hpp>
#include "tedvslam/common_include.h"

namespace tedvslam
{

    struct Frame;
    struct MapPoint;

    /// @brief 2D feature point object
    struct Feature
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        typedef std::shared_ptr<Feature> Ptr;

        std::weak_ptr<Frame> frame_;        // frame which the feature reside
        cv::KeyPoint position_;             // position of the feature
        std::weak_ptr<MapPoint> map_point_; // related map point

        bool is_outlier_ = false;      // flag for outlier
        bool is_on_left_image_ = true; // left or right image

    public:
        Feature() {}

        Feature(std::shared_ptr<Frame> frame, const cv::KeyPoint &kp)
            : frame_(frame), position_(kp) {}
    };
} // namespace myslam

#endif // FEATURE_H
