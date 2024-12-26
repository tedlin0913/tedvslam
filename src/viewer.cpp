#include "tedvslam/viewer.h"
#include "tedvslam/feature.h"
#include "tedvslam/frame.h"

#include <pangolin/pangolin.h>
#include <opencv2/opencv.hpp>

namespace tedvslam
{

    Viewer::Viewer()
    {
    }

    void Viewer::Close()
    {
    }

    // FRONTEND => VIEWER
    void Viewer::AddCurrentFrame(Frame::Ptr current_frame)
    {
        std::unique_lock<std::mutex> lck(viewer_data_mutex_);
        current_frame_ = current_frame;
    }

    void Viewer::UpdateMap()
    {
        spdlog::info("Update map");
        std::unique_lock<std::mutex> lck(viewer_data_mutex_);
        assert(map_ != nullptr);
        active_keyframes = map_->GetActiveKeyFrames();
        active_landmarks = map_->GetActiveMapPoints();
        map_updated_ = true;
    }

} // namespace myslam
