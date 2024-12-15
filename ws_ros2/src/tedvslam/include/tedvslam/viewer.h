//
// Created by gaoxiang on 19-5-4.
//

#ifndef VIEWER_H
#define VIEWER_H

#include <thread>
#include <pangolin/pangolin.h>

#include "tedvslam/common_include.h"
#include "tedvslam/frame.h"
#include "tedvslam/map.h"

namespace tedvslam
{

    class Viewer
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Viewer> Ptr;

        Viewer();

        void SetMap(Map::Ptr map) { map_ = map; }

        void Close();

        // add a frame
        void AddCurrentFrame(Frame::Ptr current_frame);

        // update map
        void UpdateMap();

    private:
        void ThreadLoop();

        void DrawFrame(Frame::Ptr frame, const float *color);

        void DrawMapPoints();

        void FollowCurrentFrame(pangolin::OpenGlRenderState &vis_camera);

        /// plot the features in current frame into an image
        cv::Mat PlotFrameImage();

        Frame::Ptr current_frame_ = nullptr;
        Map::Ptr map_ = nullptr;

        std::thread viewer_thread_;
        bool viewer_running_ = true;

        std::unordered_map<unsigned long, Frame::Ptr> active_keyframes_;
        std::unordered_map<unsigned long, MapPoint::Ptr> active_landmarks_;
        bool map_updated_ = false;

        std::mutex viewer_data_mutex_;
    };
} // namespace tedvslam

#endif
