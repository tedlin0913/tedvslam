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
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        typedef std::shared_ptr<Viewer> Ptr;

        Viewer();

        void SetMap(Map::Ptr map) { map_ = map; }

        void Close();

        // add a frame
        void AddCurrentFrame(Frame::Ptr current_frame);

        Frame::Ptr getCurrentFrame()
        {
            return current_frame_;
        }
        Map::Ptr getMap()
        {
            return map_;
        }

        // update map
        void UpdateMap();

        // void Show();

        std::unordered_map<unsigned long, KeyFrame::Ptr> active_keyframes;
        std::unordered_map<unsigned long, MapPoint::Ptr> active_landmarks;
        std::mutex viewer_data_mutex_;

    private:
        Frame::Ptr current_frame_ = nullptr;
        Map::Ptr map_ = nullptr;
        bool map_updated_ = false;
    };
} // namespace tedvslam

#endif
