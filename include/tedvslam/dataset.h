#ifndef DATASET_H
#define DATASET_H

#include "tedvslam/datasetbase.h"

namespace tedvslam
{

    /// @brief Where I should read camera frame.
    class Dataset : public DatasetBase
    {

    public:
        Dataset(const std::string &dataset_path)
            : DatasetBase(dataset_path) {}

        // initialize
        bool Init() override;

        /// create and return the next frame containing the stereo images
        Frame::Ptr NextFrame() override;

        Camera::Ptr GetCamera(int camera_id) const override
        {
            if (camera_id >= 0 && camera_id < static_cast<int>(cameras_.size()))
                return cameras_[camera_id];
            else
                throw std::out_of_range("Camera ID is out of range");
        }

    private:
        std::shared_ptr<io::CSVReader<2>> left_reader_;
        std::shared_ptr<io::CSVReader<2>> right_reader_;
    };
} // namespace tedvslam

#endif