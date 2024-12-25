#pragma once
#ifndef MAP_H
#define MAP_H

#include "tedvslam/common_include.h"
#include "tedvslam/frame.h"
#include "tedvslam/mappoint.h"
#include "tedvslam/keyframe.h"
#include "tedvslam/feature.h"
#include "tedvslam/config.h"

#include <optional>

namespace tedvslam
{
    class KeyFrame;
    class Frame;
    class MapPoint;

    class Map
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        using Ptr = std::shared_ptr<Map>;
        using KeyFramesType = std::unordered_map<unsigned long, std::shared_ptr<KeyFrame>>;
        using MapPointsType = std::unordered_map<unsigned long, std::shared_ptr<MapPoint>>;

        Map();

        // Insert a new keyframe into the map and active keyframes
        void InsertKeyFrame(const std::shared_ptr<KeyFrame> &keyframe);

        // Remove map points not observed by any active keyframe
        void RemoveOldActiveMapPoints();

        // Remove old keyframes from the active keyframes
        void RemoveOldActiveKeyframe();

        // Insert a new map point into the map
        void InsertMapPoint(const std::shared_ptr<MapPoint> &map_point);

        // Insert a new active map point into the map
        void InsertActiveMapPoint(const std::shared_ptr<MapPoint> &map_point);

        // Remove a map point from the map
        void RemoveMapPoint(const std::shared_ptr<MapPoint> &map_point);

        // Add an outlier map point to the list for later removal
        void AddOutlierMapPoint(unsigned long map_point_id);

        // Remove all outlier map points from the map
        void RemoveAllOutlierMapPoints();

        // Retrieve all map points
        MapPointsType GetAllMapPoints();

        // Retrieve all keyframes
        KeyFramesType GetAllKeyFrames();

        // Retrieve active map points
        MapPointsType GetActiveMapPoints();

        // Retrieve active keyframes
        KeyFramesType GetActiveKeyFrames();

    public:
        std::mutex map_update_mutex_; // Mutex for thread-safe map updates

    private:
        std::mutex data_mutex_;
        std::mutex outlier_map_point_mutex_;

        std::shared_ptr<KeyFrame> current_keyframe_ = nullptr;

        MapPointsType all_map_points_;
        MapPointsType active_map_points_;
        std::list<unsigned long> outlier_map_points_;

        KeyFramesType all_keyframes_;
        KeyFramesType active_keyframes_;

        int num_active_keyframes_;
    }; // namespace tedvslam
}
#endif // MAP_H
