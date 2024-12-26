#include "tedvslam/dataset.h"
#include "tedvslam/frame.h"

#include <boost/format.hpp>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
// using namespace std;

namespace tedvslam
{

    bool Dataset::Init()
    {
        try
        {
            for (int cam_id = 0; cam_id < 2; ++cam_id)
            {
                std::string yaml_path = dataset_path_ + "/cam" + std::to_string(cam_id) + "/sensor.yaml";
                YAML::Node config = YAML::LoadFile(yaml_path);
                YAML::Node T_BS = config["T_BS"];
                YAML::Node intrinsics = config["intrinsics"];
                std::vector<double> intrins = intrinsics.as<std::vector<double>>();
                std::vector<double> data = T_BS["data"].as<std::vector<double>>();
                // double baseline = config["baseline"].as<double>();
                Eigen::Matrix4d mat;
                mat << data[0], data[1], data[2], data[3],
                    data[4], data[5], data[6], data[7],
                    data[8], data[9], data[10], data[11],
                    0.0, 0.0, 0.0, 1.0;

                Eigen::Matrix3d Rcw = mat.block<3, 3>(0, 0);
                Eigen::Vector3d Tcw = mat.block<3, 1>(0, 3);

                Sophus::SE3d se3(Sophus::SO3d(Rcw), Tcw);

                double fx = intrins[0];
                double fy = intrins[1];
                double cx = intrins[2];
                double cy = intrins[3];

                Camera::Ptr new_camera = std::make_shared<Camera>(
                    fx, fy, cx, cy,
                    0.0, se3);
                cameras_.push_back(new_camera);
            }
        }
        catch (const YAML::Exception &e)
        {
            spdlog::error("Error reading YAML file: {}", e.what());
            return false;
        }

        try
        {
            for (int cam_id = 0; cam_id < 2; ++cam_id)
            {
                std::string csv_path = dataset_path_ + "/cam" + std::to_string(cam_id) + "/data.csv";
                auto reader = std::make_shared<io::CSVReader<2>>(csv_path);
                reader->read_header(io::ignore_extra_column, "timestamp [ns]", "filename");
                if (cam_id == 0)
                    left_reader_ = reader;
                else
                    right_reader_ = reader;
            }
        }
        catch (const std::exception &e)
        {
            spdlog::error("Error reading CSV file: {}", e.what());
            return false;
        }

        return true;
    }

    Frame::Ptr Dataset::NextFrame()
    {
        spdlog::info("Fetching Next Frame");

        if (!left_reader_ || !right_reader_)
        {
            spdlog::error("CSV readers are not initialized.");
            return nullptr;
        }

        Frame::Ptr new_frame = Frame::Create();

        for (int cam_id = 0; cam_id < 2; ++cam_id)
        {
            try
            {
                std::string timestamp, filename;
                auto reader = (cam_id == 0) ? left_reader_ : right_reader_;

                if (!reader->read_row(timestamp, filename))
                {
                    spdlog::warn("No more data in the CSV file for cam{}.", cam_id);
                    return nullptr;
                }

                std::string image_path = dataset_path_ + "/cam" + std::to_string(cam_id) + "/data/" + filename;
                cv::Mat image = cv::imread(image_path, cv::IMREAD_GRAYSCALE);

                if (image.empty())
                {
                    spdlog::error("Cannot find image: {}", filename);
                    return nullptr;
                }

                if (cam_id == 0)
                    new_frame->left_img_ = image;
                else
                    new_frame->right_img_ = image;
            }
            catch (const std::exception &e)
            {
                spdlog::error("Error reading image for cam{}: {}", cam_id, e.what());
                return nullptr;
            }
        }

        return new_frame;
    }

}