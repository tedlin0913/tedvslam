
#include "tedvslam/frame.h"

namespace tedvslam
{

    Frame::Frame(const cv::Mat &left_img, const cv::Mat &right_img, double timestamp)
    {
        static unsigned long factory_id = 0;

        left_img_ = left_img;
        right_img_ = right_img;
        timestamp_ = timestamp;

        frame_id_ = factory_id++;
    }

    Frame::Ptr Frame::Create(const cv::Mat &left_img, const cv::Mat &right_img, double timestamp)
    {
        return std::make_shared<Frame>(left_img, right_img, timestamp);
    }

    Frame::Ptr Frame::Create()
    {
        return std::make_shared<Frame>();
    }

    void Frame::SetPose(const SE3 &pose)
    {
        std::unique_lock<std::mutex> lock(pose_mutex_);
        pose_ = pose;
    }

    void Frame::SetRelativePose(const SE3 &relative_pose)
    {
        std::unique_lock<std::mutex> lock(relative_pose_mutex_);
        relative_pose_ = relative_pose;
    }

    SE3 Frame::Pose() const
    {
        std::unique_lock<std::mutex> lock(pose_mutex_);
        return pose_;
    }

    // ---------------------------------------------------------------------------------------------------------
    SE3 Frame::RelativePose() const
    {
        std::unique_lock<std::mutex> lock(relative_pose_mutex_);
        return relative_pose_;
    }

}
