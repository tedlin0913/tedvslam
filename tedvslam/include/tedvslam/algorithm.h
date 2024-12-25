#ifndef ALGORITHM_H
#define ALGORITHM_H

// algorithms used in myslam
#include "tedvslam/common_include.h"

namespace tedvslam
{

    /**
     * linear triangulation with SVD
     * @param poses     poses,
     * @param points    points in normalized plane
     * @param pt_world  triangulated point in the world
     * @return true if success
     */
    inline bool triangulation(const std::vector<SE3> &poses,
                              const std::vector<Vec3> &points, Vec3 &pt_world)
    {
        MatXX A(2 * poses.size(), 4);
        VecX b(2 * poses.size());
        b.setZero();
        for (size_t i = 0; i < poses.size(); ++i)
        {
            Mat34 m = poses[i].matrix3x4();
            A.block<1, 4>(2 * i, 0) = points[i][0] * m.row(2) - m.row(0);
            A.block<1, 4>(2 * i + 1, 0) = points[i][1] * m.row(2) - m.row(1);
        }
        auto svd = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
        pt_world = (svd.matrixV().col(3) / svd.matrixV()(3, 3)).head<3>();

        if (svd.singularValues()[3] / svd.singularValues()[2] < 1)
        {
            return true;
        }
        return false;
    }

    // converters
    inline Vec2 toVec2(const cv::Point2f p) { return Vec2(p.x, p.y); }

    inline Vec3 toVec3(const cv::Point3f p)
    {
        return Vec3(p.x, p.y, p.z);
    }

    // template <typename T>
    // inline std::pair<T, T> VectorMeanAndVariance(const std::vector<T> &v)
    // {
    //     // Ensure the vector is not empty to avoid division by zero
    //     if (v.empty())
    //     {
    //         throw std::invalid_argument("The input vector is empty.");
    //     }

    //     // Calculate the mean
    //     T sum = std::accumulate(v.begin(), v.end(), static_cast<T>(0));
    //     T mean = sum / static_cast<T>(v.size());

    //     // Calculate the variance
    //     T accum = 0;
    //     std::for_each(v.begin(), v.end(), [&](const T &value)
    //                   { accum += (value - mean) * (value - mean); });

    //     // Using (n - 1) for an unbiased estimator (sample standard deviation)
    //     T variance = accum / static_cast<T>(v.size() - 1);
    //     T stddev = std::sqrt(variance);

    //     return {mean, stddev};
    // }
} // namespace tedvslam

#endif // ALGORITHM_H
