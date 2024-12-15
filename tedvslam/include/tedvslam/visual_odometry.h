#pragma once
#ifndef VISUAL_ODOMETRY_H
#define VISUAL_ODOMETRY_H

#include "tedvslam/backend.h"
#include "tedvslam/common_include.h"
#include "tedvslam/dataset.h"
#include "tedvslam/frontend.h"
#include "tedvslam/viewer.h"

namespace tedvslam
{
    class VisualOdometry
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        typedef std::shared_ptr<VisualOdometry> Ptr;

        /// constructor with config file
        VisualOdometry(std::string &config_path);

        /**
         * do initialization things before run
         * @return true if success
         */
        bool Init();

        /**
         * start vo in the dataset
         */
        void Run();

        /**
         * Make a step forward in dataset
         */
        bool Step();

        void DrawFrame(Frame::Ptr frame, const float *color);

        /// get frontend status
        FrontendStatus GetFrontendStatus() const
        {
            return frontend_->GetStatus();
        }

    private:
        bool inited_ = false;
        std::string config_file_path_;

        Frontend::Ptr frontend_ = nullptr;
        Backend::Ptr backend_ = nullptr;
        Map::Ptr map_ = nullptr;
        Viewer::Ptr viewer_ = nullptr;

        // dataset
        Dataset::Ptr dataset_ = nullptr;

        // Frame::Ptr current_frame_ = nullptr;
        // Map::Ptr map_ = nullptr;

        // // std::thread viewer_thread_;
        // bool viewer_running_ = true;

        // std::unordered_map<unsigned long, Frame::Ptr> active_keyframes_;
        // std::unordered_map<unsigned long, MapPoint::Ptr> active_landmarks_;
        // bool map_updated_ = false;

        // std::mutex viewer_data_mutex_;
    };
} // namespace tedvslam

#endif // VISUAL_ODOMETRY_H
