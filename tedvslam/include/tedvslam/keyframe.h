#ifndef KEYFRAME_H
#define KEYFRAME_H

#include "tedvslam/common_include.h"

namespace tedvslam
{

    // forward declaration
    class Feature;
    class Frame;

    class KeyFrame
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        using Ptr = std::shared_ptr<KeyFrame>;

        KeyFrame() = default;

        explicit KeyFrame(const std::shared_ptr<Frame> &frame);

        static Ptr Create(const std::shared_ptr<Frame> &frame);

        void SetPose(const SE3 &pose);

        SE3 Pose() const; // Tcw

        // Retrieves all features' keypoints (not pyramid)
        std::vector<cv::KeyPoint> GetKeyPoints() const;

    public:
        unsigned long frame_id_;
        unsigned long keyframe_id_;
        double timestamp_;
        std::vector<std::shared_ptr<Feature>> features_left_;
        cv::Mat image_left_;

        // For pose graph optimization
        std::weak_ptr<KeyFrame> last_keyframe_;
        SE3 relative_pose_to_last_keyframe_;
        std::weak_ptr<KeyFrame> loop_keyframe_;
        SE3 relative_pose_to_loop_keyframe_;

        // Pyramid keypoints for computing ORB descriptors and matching
        std::vector<cv::KeyPoint> pyramid_keypoints_;

        // deeplcd::DescrVector descriptor_vector_;
        cv::Mat orb_descriptors_;

    private:
        SE3 pose_;
        mutable std::mutex pose_mutex_;
    };

} // namespace tedvslam

#endif