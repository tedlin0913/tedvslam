#pragma once
#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "tedvslam/common_include.h"

namespace tedvslam
{

    class Frame;
    class Feature;

    class MapPoint
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        using Ptr = std::shared_ptr<MapPoint>;

        MapPoint() = default;

        MapPoint(unsigned long id, const Vec3 &position);

        static Ptr Create(const Vec3 &position);

        Vec3 Pos() const
        {
            std::unique_lock<std::mutex> lock(data_mutex_);
            return position_;
        }

        void SetPos(const Vec3 &position)
        {
            std::unique_lock<std::mutex> lock(data_mutex_);
            position_ = position;
        }

        void AddActiveObservation(const std::shared_ptr<Feature> &feature);

        void AddObservation(const std::shared_ptr<Feature> &feature);

        std::list<std::weak_ptr<Feature>> GetActiveObservations() const;

        std::list<std::weak_ptr<Feature>> GetObservations() const;

        void RemoveActiveObservation(const std::shared_ptr<Feature> &feature);

        void RemoveObservation(const std::shared_ptr<Feature> &feature);

    public:
        unsigned long id_ = 0;
        int active_observed_times_ = 0;
        int observed_times_ = 0;
        bool is_outlier_ = false;

    private:
        mutable std::mutex data_mutex_;
        std::list<std::weak_ptr<Feature>> active_observations_;
        std::list<std::weak_ptr<Feature>> observations_;
        Vec3 position_ = Vec3::Zero();
    };

} // namespace tedvslam

#endif
