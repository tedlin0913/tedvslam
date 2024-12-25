#include "tedvslam/keyframe.h"

#include "tedvslam/frame.h"
#include "tedvslam/feature.h"
#include "tedvslam/mappoint.h"

namespace tedvslam
{

    KeyFrame::KeyFrame(const std::shared_ptr<Frame> &frame)
    {
        static unsigned long factory_id = 0;
        keyframe_id_ = factory_id++;

        // Copy members from Frame
        frame_id_ = frame->frame_id_;
        // timestamp_ = frame->timestamp_;
        image_left_ = frame->left_img_;
        features_left_ = frame->features_left_;

        for (size_t i = 0; i < features_left_.size(); ++i)
        {
            auto map_point = features_left_[i]->map_point_.lock();
            if (map_point)
            {
                features_left_[i]->map_point_ = map_point;
            }
        }
    }

    KeyFrame::Ptr KeyFrame::Create(const std::shared_ptr<Frame> &frame)
    {
        auto new_keyframe = std::make_shared<KeyFrame>(frame);

        // Link Feature->mpKF to the current keyframe
        for (size_t i = 0; i < new_keyframe->features_left_.size(); ++i)
        {
            auto feature = new_keyframe->features_left_[i];
            feature->keyframe_ = new_keyframe;

            auto map_point = feature->map_point_.lock();
            if (map_point)
            {
                map_point->AddObservation(feature);
            }
        }

        return new_keyframe;
    }

    std::vector<cv::KeyPoint> KeyFrame::GetKeyPoints() const
    {
        std::vector<cv::KeyPoint> keypoints(features_left_.size());
        for (size_t i = 0; i < features_left_.size(); ++i)
        {
            keypoints[i] = features_left_[i]->keypoint_;
        }
        return keypoints;
    }

    void KeyFrame::SetPose(const SE3 &pose)
    {
        std::unique_lock<std::mutex> lock(pose_mutex_);
        pose_ = pose;
    }

    SE3 KeyFrame::Pose() const
    {
        std::unique_lock<std::mutex> lock(pose_mutex_);
        return pose_;
    }

} // namespace tedvslam