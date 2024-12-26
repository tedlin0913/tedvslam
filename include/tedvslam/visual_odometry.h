#pragma once
#ifndef VISUAL_ODOMETRY_H
#define VISUAL_ODOMETRY_H

#include <thread>

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

        bool Init();

        void Run();

        bool Step();

        void DrawFrame(Frame::Ptr frame, const float *color);
        void DrawFrame(KeyFrame::Ptr frame, const float *color);
        void DrawMapPoints();

        void FollowCurrentFrame(pangolin::OpenGlRenderState &vis_camera);
        cv::Mat PlotFrameImage();

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
    };
} // namespace tedvslam

#endif // VISUAL_ODOMETRY_H
