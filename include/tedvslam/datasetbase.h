#ifndef DATASETBASE_H
#define DATASETBASE_H

#include "tedvslam/camera.h"
#include "tedvslam/common_include.h"
#include "tedvslam/frame.h"
#include "csv.h"

namespace tedvslam
{
    class DatasetBase
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        typedef std::shared_ptr<DatasetBase> Ptr;

        DatasetBase(const std::string &dataset_path)
            : dataset_path_(dataset_path) {}

        virtual ~DatasetBase() = default;

        // Initialize the dataset
        virtual bool Init() = 0;

        // Create and return the next frame containing the stereo images
        virtual Frame::Ptr NextFrame() = 0;

        // Get camera by ID
        virtual Camera::Ptr GetCamera(int camera_id) const = 0;

    protected:
        std::string dataset_path_;
        int current_image_index_ = 0;
        std::vector<Camera::Ptr> cameras_;
    };
}

#endif