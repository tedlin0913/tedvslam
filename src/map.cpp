#include "tedvslam/map.h"

namespace tedvslam
{

    Map::Map()
    {
        num_active_keyframes_ = Config::Get<int>("Map.activeMap.size").value_or(0);
    }

    void Map::InsertKeyFrame(const std::shared_ptr<KeyFrame> &keyframe)
    {
        spdlog::info("insert kf to map");
        current_keyframe_ = keyframe;
        {
            std::unique_lock<std::mutex> lock(data_mutex_);
            if (all_keyframes_.find(keyframe->keyframe_id_) == all_keyframes_.end())
            {
                all_keyframes_[keyframe->keyframe_id_] = keyframe;
                active_keyframes_[keyframe->keyframe_id_] = keyframe;
            }
            else
            {
                all_keyframes_[keyframe->keyframe_id_] = keyframe;
                active_keyframes_[keyframe->keyframe_id_] = keyframe;
            }
        }

        for (const auto &feature : keyframe->features_left_)
        {
            auto map_point = feature->map_point_.lock();
            if (map_point)
            {
                map_point->AddActiveObservation(feature);
                InsertActiveMapPoint(map_point);
            }
        }
        spdlog::info("start remove old frames");
        if (active_keyframes_.size() > static_cast<size_t>(num_active_keyframes_))
        {
            RemoveOldActiveKeyframe();
            RemoveOldActiveMapPoints();
        }
        spdlog::info("done remove old frames");
    }

    void Map::InsertMapPoint(const std::shared_ptr<MapPoint> &map_point)
    {
        std::unique_lock<std::mutex> lock(data_mutex_);
        all_map_points_[map_point->id_] = map_point;
    }

    void Map::InsertActiveMapPoint(const std::shared_ptr<MapPoint> &map_point)
    {
        std::unique_lock<std::mutex> lock(data_mutex_);
        active_map_points_[map_point->id_] = map_point;
    }

    void Map::RemoveOldActiveKeyframe()
    {
        std::unique_lock<std::mutex> lock(data_mutex_);

        if (!current_keyframe_)
            return;

        double max_distance = 0, min_distance = 9999;
        unsigned long max_kf_id = 0, min_kf_id = 0;

        auto current_pose_inv = current_keyframe_->Pose().inverse();
        for (const auto &keyframe : active_keyframes_)
        {
            if (keyframe.second == current_keyframe_)
                continue;

            double distance = (keyframe.second->Pose() * current_pose_inv).log().norm();
            if (distance > max_distance)
            {
                max_distance = distance;
                max_kf_id = keyframe.first;
            }
            else if (distance < min_distance)
            {
                min_distance = distance;
                min_kf_id = keyframe.first;
            }
        }

        const double min_distance_threshold = 0.2;
        auto frame_to_remove = (min_distance < min_distance_threshold)
                                   ? active_keyframes_[min_kf_id]
                                   : active_keyframes_[max_kf_id];

        active_keyframes_.erase(frame_to_remove->keyframe_id_);
        for (const auto &feature : frame_to_remove->features_left_)
        {
            auto map_point = feature->map_point_.lock();
            if (map_point)
            {
                map_point->RemoveActiveObservation(feature);
            }
        }
    }

    void Map::RemoveOldActiveMapPoints()
    {
        std::unique_lock<std::mutex> lock(data_mutex_);

        for (auto iter = active_map_points_.begin(); iter != active_map_points_.end();)
        {
            if (iter->second->active_observed_times_ == 0)
            {
                iter = active_map_points_.erase(iter);
            }
            else
            {
                ++iter;
            }
        }
    }

    void Map::RemoveMapPoint(const std::shared_ptr<MapPoint> &map_point)
    {
        std::unique_lock<std::mutex> lock(data_mutex_);
        all_map_points_.erase(map_point->id_);
        active_map_points_.erase(map_point->id_);
    }

    void Map::AddOutlierMapPoint(unsigned long map_point_id)
    {
        std::unique_lock<std::mutex> lock(outlier_map_point_mutex_);
        outlier_map_points_.push_back(map_point_id);
    }

    void Map::RemoveAllOutlierMapPoints()
    {
        std::unique_lock<std::mutex> lock(data_mutex_);
        std::unique_lock<std::mutex> lock_outlier(outlier_map_point_mutex_);

        for (const auto &map_point_id : outlier_map_points_)
        {
            all_map_points_.erase(map_point_id);
            active_map_points_.erase(map_point_id);
        }
        outlier_map_points_.clear();
    }

    Map::MapPointsType Map::GetAllMapPoints()
    {
        std::unique_lock<std::mutex> lock(data_mutex_);
        return all_map_points_;
    }

    Map::KeyFramesType Map::GetAllKeyFrames()
    {
        std::unique_lock<std::mutex> lock(data_mutex_);
        return all_keyframes_;
    }

    Map::MapPointsType Map::GetActiveMapPoints()
    {
        std::unique_lock<std::mutex> lock(data_mutex_);
        return active_map_points_;
    }

    Map::KeyFramesType Map::GetActiveKeyFrames()
    {
        std::unique_lock<std::mutex> lock(data_mutex_);
        return active_keyframes_;
    }

} // namespace tedvslam
