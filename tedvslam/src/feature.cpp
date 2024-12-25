#include "tedvslam/feature.h"

namespace tedvslam
{
    Feature::Feature(const std::shared_ptr<KeyFrame> &keyframe, const cv::KeyPoint &keypoint)
    {
        keyframe_ = keyframe;
        keypoint_ = keypoint;
    }

    Feature::Feature(const cv::KeyPoint &keypoint)
    {
        keypoint_ = keypoint;
    }

    Feature::Ptr Feature::Create(const cv::KeyPoint &keypoint)
    {
        return std::make_shared<Feature>(keypoint);
    }

    Feature::Ptr Feature::Create(const std::shared_ptr<KeyFrame> &keyframe, const cv::KeyPoint &keypoint)
    {
        return std::make_shared<Feature>(keyframe, keypoint);
    }

}