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

    private:
        void RunBackend();

        void OptimizeActiveMap();

    private:
        std::shared_ptr<Camera> camera_left_;
        std::shared_ptr<Camera> camera_right_;
        std::shared_ptr<Map> map_;
        std::shared_ptr<Viewer> viewer_ = nullptr;

        std::thread backend_thread_;
        std::atomic<bool> is_running_{false};

        std::condition_variable map_update_;
        std::mutex new_keyframe_mutex_;
        std::mutex stop_mutex_;
        bool need_optimization_ = false;

        std::list<std::shared_ptr<KeyFrame>> new_keyframes_;

        std::shared_ptr<KeyFrame> current_keyframe_ = nullptr;
    };

} // namespace tedvslam

#endif // BACKEND_H