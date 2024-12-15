#ifndef DATASET_H
#define DATASET_H
#include "tedvslam/camera.h"
#include "tedvslam/common_include.h"
#include "tedvslam/frame.h"

namespace tedvslam
{

    /// @brief Where I should read camera frame.
    class Dataset
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        typedef std::shared_ptr<Dataset> Ptr;
        Dataset(const std::string &dataset_path);

        // initialize
        bool Init();

        /// create and return the next frame containing the stereo images
        Frame::Ptr NextFrame();

        /// get camera by id
        Camera::Ptr GetCamera(int camera_id) const
        {
            return cameras_.at(camera_id);
        }

    private:
        std::string dataset_path_;
        int current_image_index_ = 0;

        std::vector<Camera::Ptr> cameras_;
    };
} // namespace tedvslam

#endif