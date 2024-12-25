#ifndef BACKEND_H
#define BACKEND_H

#include "tedvslam/common_include.h"
#include "tedvslam/frame.h"
#include "tedvslam/map.h"
#include "tedvslam/camera.h"
#include "tedvslam/viewer.h"
#include "tedvslam/keyframe.h"

namespace tedvslam
{
    class Camera;
    class Viewer;
    class Map;
    class Frame;
    class KeyFrame;

    class Backend
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        using Ptr = std::shared_ptr<Backend>;

        Backend();

        //! Stop the backend thread
        void Stop();

        //! Set the viewer instance
        void SetViewer(const std::shared_ptr<Viewer> &viewer)
        {
            viewer_ = viewer;
        }

        void UpdateMap();

        // //! Set the loop closing instance
        // void SetLoopClosing(const std::shared_ptr<LoopClosing> &loop_closing)
        // {
        //     loop_closing_ = loop_closing;
        // }

        //! Set the left and right cameras
        void SetCameras(const std::shared_ptr<Camera> &left, const std::shared_ptr<Camera> &right)
        {
            camera_left_ = left;
            camera_right_ = right;
        }

        //! Set the map instance
        void SetMap(const std::shared_ptr<Map> &map)
        {
            map_ = map;
        }

        //! Insert a keyframe into the backend for processing
        // void InsertKeyFrame(const std::shared_ptr<KeyFrame> &keyframe);

        // //! Request the backend thread to pause
        // void RequestPause();

        // //! Check if the backend thread is paused
        // bool IsPaused() const;

        // //! Resume the backend thread
        // void Resume();

    private:
        //! Check if there are new keyframes to process
        // bool HasNewKeyFrames() const;

        // bool CheckNewKeyFrames();

        // //! Process a new keyframe: insert into the map and loop closing
        // void ProcessNewKeyFrame();

        //! Main loop of the backend thread
        void RunBackend();

        //! Optimize the active map using g2o
        void OptimizeActiveMap();

    private:
        std::shared_ptr<Camera> camera_left_;
        std::shared_ptr<Camera> camera_right_;
        std::shared_ptr<Map> map_;
        std::shared_ptr<Viewer> viewer_ = nullptr;
        // std::shared_ptr<LoopClosing> loop_closing_ = nullptr;

        std::thread backend_thread_;
        std::atomic<bool> is_running_{false};
        // std::atomic<bool> request_pause_{false};
        // std::atomic<bool> is_paused_{false};
        std::condition_variable map_update_;
        std::mutex new_keyframe_mutex_;
        std::mutex stop_mutex_;
        bool need_optimization_ = false;

        std::list<std::shared_ptr<KeyFrame>> new_keyframes_;

        std::shared_ptr<KeyFrame> current_keyframe_ = nullptr;
    };

} // namespace tedvslam

#endif // BACKEND_H