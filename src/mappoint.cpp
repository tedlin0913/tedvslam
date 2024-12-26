#include "tedvslam/mappoint.h"
#include "tedvslam/feature.h"

namespace tedvslam
{

    MapPoint::MapPoint(unsigned long id, const Vec3 &position)
    {
        position_ = position;
        id_ = id;
    }

    MapPoint::Ptr MapPoint::Create(const Vec3 &position)
    {
        static unsigned long factory_id = 0;
        auto new_map_point = std::make_shared<MapPoint>(factory_id++, position);
        return new_map_point;
    }

    void MapPoint::AddObservation(const std::shared_ptr<Feature> &feature)
    {
        std::unique_lock<std::mutex> lock(data_mutex_);
        observations_.push_back(feature);
        observed_times_++;
    }

    void MapPoint::AddActiveObservation(const std::shared_ptr<Feature> &feature)
    {
        std::unique_lock<std::mutex> lock(data_mutex_);
        active_observations_.push_back(feature);
        active_observed_times_++;
    }

    void MapPoint::RemoveActiveObservation(const std::shared_ptr<Feature> &feature)
    {
        std::unique_lock<std::mutex> lock(data_mutex_);
        for (auto iter = active_observations_.begin(); iter != active_observations_.end(); ++iter)
        {
            if (iter->lock() == feature)
            {
                active_observations_.erase(iter);
                active_observed_times_--;
                break;
            }
        }
    }

    void MapPoint::RemoveObservation(const std::shared_ptr<Feature> &feature)
    {
        std::unique_lock<std::mutex> lock(data_mutex_);
        for (auto iter = observations_.begin(); iter != observations_.end(); ++iter)
        {
            if (iter->lock() == feature)
            {
                observations_.erase(iter);
                feature->map_point_.reset();
                observed_times_--;
                break;
            }
        }
    }

    std::list<std::weak_ptr<Feature>> MapPoint::GetActiveObservations() const
    {
        std::unique_lock<std::mutex> lock(data_mutex_);
        return active_observations_;
    }

    std::list<std::weak_ptr<Feature>> MapPoint::GetObservations() const
    {
        std::unique_lock<std::mutex> lock(data_mutex_);
        return observations_;
    }

} // namespace tedvslam
