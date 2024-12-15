#pragma once
#ifndef MAP_H
#define MAP_H

#include "tedvslam/common_include.h"
#include "tedvslam/frame.h"
#include "tedvslam/mappoint.h"

namespace tedvslam
{

    /// Interaction with fronend: InsertKeyframe and InsertMapPoint
    /// Backend: check and remove outliers
    class Map
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        typedef std::shared_ptr<Map> Ptr;
        typedef std::unordered_map<unsigned long, MapPoint::Ptr> LandmarksType;
        typedef std::unordered_map<unsigned long, Frame::Ptr> KeyframesType;

        Map() {}

        /// add a keyframe to map
        void InsertKeyFrame(Frame::Ptr frame);
        /// add a map point to map
        void InsertMapPoint(MapPoint::Ptr map_point);

        /// all mappoints
        LandmarksType GetAllMapPoints()
        {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return landmarks_;
        }
        /// all keyframes
        KeyframesType GetAllKeyFrames()
        {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return keyframes_;
        }

        /// active map points
        LandmarksType GetActiveMapPoints()
        {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return active_landmarks_;
        }

        /// get active key frames
        KeyframesType GetActiveKeyFrames()
        {
            std::unique_lock<std::mutex> lck(data_mutex_);
            return active_keyframes_;
        }

        /// clean up the map
        void CleanMap();

    private:
        void RemoveOldKeyframe();

        std::mutex data_mutex_;
        LandmarksType landmarks_;        // all landmarks
        LandmarksType active_landmarks_; // active landmarks
        KeyframesType keyframes_;        // all key-frames
        KeyframesType active_keyframes_; // all key-frames

        Frame::Ptr current_frame_ = nullptr;

        // settings
        int num_active_keyframes_ = 7;
    };
} // namespace tedvslam

#endif // MAP_H
