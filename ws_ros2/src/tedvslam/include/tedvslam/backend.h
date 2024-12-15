#ifndef BACKEND_H
#define BACKEND_H

#include "tedvslam/common_include.h"
#include "tedvslam/frame.h"
#include "tedvslam/map.h"

namespace tedvslam
{
    class Map;

    // Optimize on a seperate thread.
    class Backend
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Backend> Ptr;

        Backend();

        // Setup cameras, for calibration (internal and external parameters)
        // For monocular, use only left?
        void SetCameras(Camera::Ptr left, Camera::Ptr right)
        {
            cam_left_ = left;
            cam_right_ = right;
        }

        /// Setup map
        void SetMap(std::shared_ptr<Map> map) { map_ = map; }

        /// Update map and do optimization
        void UpdateMap();

        /// Stop thread
        void Stop();

    private:
        /// Backend thread loop
        void BackendLoop();

        /// optimize keyframe and landmark (local optimization)
        void Optimize(Map::KeyframesType &keyframes, Map::LandmarksType &landmarks);

        std::shared_ptr<Map> map_;
        std::thread backend_thread_;
        std::mutex data_mutex_;

        std::condition_variable map_update_;
        std::atomic<bool> backend_running_;

        Camera::Ptr cam_left_ = nullptr, cam_right_ = nullptr;
    };

} // namespace tedvslam

#endif // BACKEND_H