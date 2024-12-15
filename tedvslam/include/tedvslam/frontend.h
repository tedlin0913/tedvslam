#pragma once
#ifndef FRONTEND_H
#define FRONTEND_H

#include <opencv2/features2d.hpp>

#include "tedvslam/common_include.h"
#include "tedvslam/frame.h"
#include "tedvslam/map.h"

namespace tedvslam
{

    class Backend;
    class Viewer;

    enum class FrontendStatus
    {
        INITING,
        TRACKING_GOOD,
        TRACKING_BAD,
        LOST
    };

    /// @brief Estimate pose, if frame can be a keyframe, trigger optimization
    class Frontend
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        typedef std::shared_ptr<Frontend> Ptr;

        Frontend();

        /// Add a frame and calculated posse
        bool AddFrame(Frame::Ptr frame);

        /// Set map
        void SetMap(Map::Ptr map) { map_ = map; }

        /// @brief set backend
        /// @param backend
        void SetBackend(std::shared_ptr<Backend> backend) { backend_ = backend; }

        /// @brief Set viewer
        /// @param viewer
        void SetViewer(std::shared_ptr<Viewer> viewer) { viewer_ = viewer; }

        /// @brief Get frontend status
        /// @return
        FrontendStatus GetStatus() const { return status_; }

        void SetCameras(Camera::Ptr left, Camera::Ptr right)
        {
            camera_left_ = left;
            camera_right_ = right;
        }

    private:
        /**
         * Track in normal mode
         * @return true if success
         */
        bool Track();

        /**
         * Reset when lost
         * @return true if success
         */
        bool Reset();

        /**
         * Track with last frame
         * @return num of tracked points
         */
        int TrackLastFrame();

        /**
         * estimate current frame's pose
         * @return num of inliers
         */
        int EstimateCurrentPose();

        /**
         * set current frame as a keyframe and insert it into backend
         * @return true if success
         */
        bool InsertKeyframe();

        /**
         * Try init the frontend with stereo images saved in current_frame_
         * @return true if success
         */
        bool StereoInit();

        /**
         * Detect features in left image in current_frame_
         * keypoints will be saved in current_frame_
         * @return
         */
        int DetectFeatures();

        /**
         * Find the corresponding features in right image of current_frame_
         * @return num of features found
         */
        int FindFeaturesInRight();

        /**
         * Build the initial map with single image
         * @return true if succeed
         */
        bool BuildInitMap();

        /**
         * Triangulate the 2D points in current frame
         * @return num of triangulated points
         */
        int TriangulateNewPoints();

        /**
         * Set the features in keyframe as new observation of the map points
         */
        void SetObservationsForKeyFrame();

        // data
        FrontendStatus status_ = FrontendStatus::INITING;

        Frame::Ptr current_frame_ = nullptr; // current frame
        Frame::Ptr last_frame_ = nullptr;    // last frame
        Camera::Ptr camera_left_ = nullptr;  // left cam
        Camera::Ptr camera_right_ = nullptr; // right cam

        Map::Ptr map_ = nullptr;
        std::shared_ptr<Backend> backend_ = nullptr;
        std::shared_ptr<Viewer> viewer_ = nullptr;

        /// relative motion between current frame and last frame
        /// used to estimate initial param of current frame
        SE3 relative_motion_;

        /// testing new keyframes
        int tracking_inliers_ = 0;

        // params
        int num_features_ = 200;
        int num_features_init_ = 100;
        int num_features_tracking_ = 50;
        int num_features_tracking_bad_ = 20;
        int num_features_needed_for_keyframe_ = 80;

        // utilities
        cv::Ptr<cv::GFTTDetector> gftt_; // feature detector in opencv
    };

} // namespace tedvslam

#endif // FRONTEND_H
