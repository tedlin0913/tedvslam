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

    Dataset::Dataset(const std::string &dataset_path)
        : dataset_path_(dataset_path) {}

    bool Dataset::Init()
    {
        try
        {
            YAML::Node config = YAML::LoadFile(dataset_path_ + "/cam0/sensor.yaml");
            YAML::Node T_BS = config["T_BS"];
            int rows = T_BS["rows"].as<int>();
            int cols = T_BS["cols"].as<int>();
            std::vector<double> data = T_BS["data"].as<std::vector<double>>();
            std::cout << "T_BS Matrix (" << rows << "x" << cols << "):" << std::endl;

            Mat33 K;
            K << data[0], data[1], data[2],
                data[4], data[5], data[6],
                data[8], data[9], data[10];

            Vec3 t;
            t << data[3], data[7], data[11];
            t = K.inverse() * t;
            K = K * 0.5;
            Camera::Ptr new_camera(new Camera(K(0, 0), K(1, 1), K(0, 2), K(1, 2),
                                              t.norm(), SE3(SO3(), t)));
            cameras_.push_back(new_camera);
        }
        catch (const YAML::Exception &e)
        {
            spdlog::error("Error reading YAML file: {}", e.what());
            // std::cerr << "Error reading YAML file: " << e.what() << std::endl;
            return 1;
        }

        try
        {
            YAML::Node config = YAML::LoadFile(dataset_path_ + "/cam1/sensor.yaml");
            YAML::Node T_BS = config["T_BS"];
            int rows = T_BS["rows"].as<int>();
            int cols = T_BS["cols"].as<int>();
            std::vector<double> data = T_BS["data"].as<std::vector<double>>();
            std::cout << "T_BS Matrix (" << rows << "x" << cols << "):" << std::endl;

            Mat33 K;
            K << data[0], data[1], data[2],
                data[4], data[5], data[6],
                data[8], data[9], data[10];

            Vec3 t;
            t << data[3], data[7], data[11];
            t = K.inverse() * t;
            K = K * 0.5;
            Camera::Ptr new_camera(new Camera(K(0, 0), K(1, 1), K(0, 2), K(1, 2),
                                              t.norm(), SE3(SO3(), t)));
            cameras_.push_back(new_camera);
        }
        catch (const YAML::Exception &e)
        {
            spdlog::error("Error reading YAML file: {}", e.what());
            // std::cerr << "Error reading YAML file: " << e.what() << std::endl;
            return 1;
        }

        try
        {
            // Initialize the CSV reader
            left_reader_ = std::make_shared<io::CSVReader<2>>(dataset_path_ + "/cam0/data.csv");
            left_reader_->read_header(io::ignore_extra_column, "timestamp [ns]", "filename");
        }
        catch (const std::exception &e)
        {
            spdlog::error("Error reading CSV file: {}", e.what());
            // std::cerr << "Error reading CSV file: " << e.what() << std::endl;
            return false;
        }

        try
        {
            // Initialize the CSV reader
            right_reader_ = std::make_shared<io::CSVReader<2>>(dataset_path_ + "/cam1/data.csv");
            right_reader_->read_header(io::ignore_extra_column, "timestamp [ns]", "filename");
        }
        catch (const std::exception &e)
        {
            spdlog::error("Error reading CSV file: {}", e.what());
            // std::cerr << "Error reading CSV file: " << e.what() << std::endl;
            return false;
        }
        // // read camera intrinsics and extrinsics
        // std::ifstream fin(dataset_path_ + "/cam0/sensor.yaml");
        // if (!fin)
        // {
        //     // LOG(ERROR) << "cannot find " << dataset_path_ << "/calib.txt!";
        //     return false;
        // }

        // for (int i = 0; i < 4; ++i)
        // {
        //     char camera_name[3];
        //     for (int k = 0; k < 3; ++k)
        //     {
        //         fin >> camera_name[k];
        //     }
        //     double projection_data[12];
        //     for (int k = 0; k < 12; ++k)
        //     {
        //         fin >> projection_data[k];
        //     }
        //     Mat33 K;
        //     K << projection_data[0], projection_data[1], projection_data[2],
        //         projection_data[4], projection_data[5], projection_data[6],
        //         projection_data[8], projection_data[9], projection_data[10];
        //     Vec3 t;
        //     t << projection_data[3], projection_data[7], projection_data[11];
        //     t = K.inverse() * t;
        //     K = K * 0.5;
        //     Camera::Ptr new_camera(new Camera(K(0, 0), K(1, 1), K(0, 2), K(1, 2),
        //                                       t.norm(), SE3(SO3(), t)));
        //     cameras_.push_back(new_camera);
        //     // LOG(INFO) << "Camera " << i << " extrinsics: " << t.transpose();
        // }
        // fin.close();
        // current_image_index_ = 0;
        return true;
    }

    Frame::Ptr Dataset::NextFrame()
    {
        if (!left_reader_ || !right_reader_)
        {
            spdlog::error("CSV reader is not initialized.");
            // std::cerr << "CSV reader is not initialized." << std::endl;
            return nullptr;
        }

        Frame::Ptr new_frame = Frame::CreateFrame();
        // boost::format fmt("%s/image_%d/%06d.png");
        try
        {
            std::string timestamp, filename;
            if (!left_reader_->read_row(timestamp, filename))
            {
                spdlog::warn("No more data in the CSV file.");
                // std::cerr << "No more rows in the CSV file." << std::endl;
                return nullptr;
            }

            // Construct file paths for left and right images
            std::string image_path = dataset_path_ + "/cam0/data/" + filename;

            // Load the images
            cv::Mat image = cv::imread(image_path, cv::IMREAD_GRAYSCALE);

            if (image.empty())
            {
                spdlog::error("Cannot find images: {}", filename);
                // std::cerr << "Cannot find images: " << filename << std::endl;
                return nullptr;
            }

            // Resize the images
            cv::Mat image_resized;
            cv::resize(image, image_resized, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);
            // cv::resize(image_right, image_right_resized, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);

            // Create and return a new frame

            new_frame->left_img_ = image_resized;
        }
        catch (const std::exception &e)
        {
            spdlog::error("Error reading left image: {}", e.what());
            // std::cerr << "Error reading CSV file: " << e.what() << std::endl;
            return nullptr;
        }

        try
        {
            std::string timestamp, filename;
            if (!right_reader_->read_row(timestamp, filename))
            {
                spdlog::warn("No more data in the CSV file.");
                // std::cerr << "No more rows in the CSV file." << std::endl;
                return nullptr;
            }

            // Construct file paths for right and right images
            std::string image_path = dataset_path_ + "/cam1/data/" + filename;

            // Load the images
            cv::Mat image = cv::imread(image_path, cv::IMREAD_GRAYSCALE);

            if (image.empty())
            {
                spdlog::error("Cannot find images: {}", filename);
                // std::cerr << "Cannot find images: " << filename << std::endl;
                return nullptr;
            }

            // Resize the images
            cv::Mat image_resized;
            cv::resize(image, image_resized, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);
            // cv::resize(image_right, image_right_resized, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);

            // Create and return a new frame

            new_frame->right_img_ = image_resized;
        }
        catch (const std::exception &e)
        {
            spdlog::error("Error reading right image: {}", e.what());
            // std::cerr << "Error reading CSV file: " << e.what() << std::endl;
            return nullptr;
        }

        return new_frame;

        // cv::Mat image_left, image_right;

        // image_left =
        //     cv::imread((fmt % dataset_path_ % 0 % current_image_index_).str(),
        //                cv::IMREAD_GRAYSCALE);
        // image_right =
        //     cv::imread((fmt % dataset_path_ % 1 % current_image_index_).str(),
        //                cv::IMREAD_GRAYSCALE);

        // if (image_left.data == nullptr || image_right.data == nullptr)
        // {
        //     // LOG(WARNING) << "cannot find images at index " << current_image_index_;
        //     return nullptr;
        // }

        // cv::Mat image_left_resized, image_right_resized;
        // cv::resize(image_left, image_left_resized, cv::Size(), 0.5, 0.5,
        //            cv::INTER_NEAREST);
        // cv::resize(image_right, image_right_resized, cv::Size(), 0.5, 0.5,
        //            cv::INTER_NEAREST);

        // auto new_frame = Frame::CreateFrame();
        // new_frame->left_img_ = image_left_resized;
        // new_frame->right_img_ = image_right_resized;
        // current_image_index_++;
        // return new_frame;
    }

} // namespace tedvslam