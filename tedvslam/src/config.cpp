#include "tedvslam/config.h"
#include <iostream>
#include <yaml-cpp/yaml.h>

namespace tedvslam
{
    bool Config::SetParameterFile(const std::string &filename)
    {
        try
        {
            // Load the YAML file
            // file_ = YAML::LoadFile(filename);

            if (config_ == nullptr)
                config_ = std::shared_ptr<Config>(new Config);
            config_->file_ = YAML::LoadFile(filename);
            // if (config_->file_.isOpened() == false)
            // {
            //     LOG(ERROR) << "parameter file " << filename << " does not exist.";
            //     config_->file_.release();
            //     return false;
            // }
            // return true;

            // // Read the dataset directory
            // std::string dataset_dir = file_["dataset_dir"].as<std::string>();
            // std::cout << "Dataset Directory: " << dataset_dir << std::endl;

            // // Read the number of features
            // int num_features = file_["num_features"].as<int>();
            // int num_features_init = file_["num_features_init"].as<int>();
            // int num_features_tracking = file_["num_features_tracking"].as<int>();

            // std::cout << "Number of Features: " << num_features << std::endl;
            // std::cout << "Number of Initial Features: " << num_features_init << std::endl;
            // std::cout << "Number of Tracking Features: " << num_features_tracking << std::endl;
        }
        catch (const YAML::Exception &e)
        {
            spdlog::info("Error reading YAML file: {}", e.what());
            // std::cerr << "Error reading YAML file: " << e.what() << std::endl;
            return false;
        }
        spdlog::info("Done setting parameter file");
        return true;
        // std::cout << "Set Parameter File" << std::endl;
        // if (config_ == nullptr)
        //     config_ = std::shared_ptr<Config>(new Config);
        // config_->file_ = cv::FileStorage(filename.c_str(), cv::FileStorage::READ);
        // if (config_->file_.isOpened() == false)
        // {
        //     std::cout << "parameter file " << filename << " does not exist.";
        //     config_->file_.release();
        //     return false;
        // }
        // return true;
    }

    Config::~Config()
    {
        // if (file_.isOpened())
        //     file_.release();
    }
    std::shared_ptr<Config> Config::config_ = nullptr;
}
