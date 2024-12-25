#pragma once
#ifndef FRONTEND_H
#define FRONTEND_H

#include <memory>

#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>

#include "tedvslam/common_include.h"
#include "tedvslam/g2o_types.h"
#include "tedvslam/frame.h"
#include "tedvslam/map.h"
#include "tedvslam/backend.h"
#include "tedvslam/config.h"
#include "tedvslam/ORBextractor.h"
#include "tedvslam/viewer.h"
#include "tedvslam/camera.h"
#include "tedvslam/keyframe.h"
#include "tedvslam/feature.h"
#include "tedvslam/algorithm.h"

namespace tedvslam
{
    class Frame;
    class Viewer;
    class ORBextractor;
    class Camera;
    class Map;
    class Backend;
    class KeyFrame;

    // Four tracking statuses
    enum class FrontendStatus
    {
        INITING,
        TRACKING_GOOD,
        TRACKING_BAD,
        LOST
    };

    class Frontend
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        using Ptr = std::shared_ptr<Frontend>;

        Frontend();

        bool AddFrame(Frame::Ptr frame);

        // Process new pair of images
        // Returns true if it runs normally.
        // bool GrabStereoImage(const cv::Mat &left_img, const cv::Mat &right_img, double timestamp);

        void SetViewer(std::shared_ptr<Viewer> viewer)
        {
            viewer_ = viewer;
        }

        void SetCameras(std::shared_ptr<Camera> left, std::shared_ptr<Camera> right)
        {
            camera_left_ = left;
            camera_right_ = right;
        }

        void SetMap(std::shared_ptr<Map> map)
        {
            map_ = map;
        }

        void SetBackend(std::shared_ptr<Backend> backend)
        {
            backend_ = backend;
        }

        void SetORBextractor(std::shared_ptr<ORBextractor> orb)
        {
            orb_extractor_ = orb;
        }

        FrontendStatus GetStatus() const
        {
            return status_;
        }

        void SetObservationsForKeyFrame();

    private:
        // Tracking initialization. Returns true if successful.
        bool StereoInit();

        // Build the initial map. Returns true if successful.
        bool BuildInitMap();

        // Standard tracking. Returns true if successful.
        bool Track();

        // Estimate the current frame's pose using motion model.
        // Returns the number of good tracked points.
        int TrackLastFrame();

        // Optimize the current frame's pose using g2o.
        // Returns the inlier numbers.
        int EstimateCurrentPose();

        // Detect new features in the current image (left).
        // Returns the number of features.
        int DetectFeatures();

        // Find the corresponding features in the right image of the current frame.
        // Returns the number of corresponding features found.
        int FindFeaturesInRight();

        // Create a new keyframe (based on the current frame) and update.
        // Returns true if successful.
        bool InsertKeyFrame();

        // Create new map points and insert them into the map.
        // Returns the number of new map points created.
        int TriangulateNewPoints();

        // Reset
        bool Reset();

        std::shared_ptr<ORBextractor> orb_extractor_, orb_extractor_init_;
        std::shared_ptr<Camera> camera_left_, camera_right_;
        std::shared_ptr<Map> map_;
        std::shared_ptr<Backend> backend_;
        std::shared_ptr<Viewer> viewer_;

        FrontendStatus status_ = FrontendStatus::INITING;

        std::shared_ptr<Frame> current_frame_;
        std::shared_ptr<Frame> last_frame_;
        std::shared_ptr<KeyFrame> reference_kf_;

        SE3 relative_motion_;
        SE3 relative_motion_to_reference_kf_;

        cv::Mat dist_coeffs0_, dist_coeffs1_;

        int num_features_tracking_good_;
        int num_features_tracking_bad_;
        int num_features_init_good_;

        bool need_undistortion_;

        int tracking_inliers_;

        int num_features_init_;

        cv::Ptr<cv::ORB> orb_;

        cv::Ptr<cv::GFTTDetector> gftt_;
    };
} // namespace tedvslam

#endif // FRONTEND_H
