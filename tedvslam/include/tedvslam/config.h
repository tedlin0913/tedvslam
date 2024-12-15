#pragma once
#ifndef CONFIG_H
#define CONFIG_H

#include "tedvslam/common_include.h"

namespace tedvslam
{
    class Config
    {
    private:
        static std::shared_ptr<Config> config_;
        cv::FileStorage file_;

        Config() {} // private constructor makes a singleton
    public:
        ~Config(); // close the file when deconstructing

        // set a new config file
        static bool SetParameterFile(const std::string &filename);

        // access the parameter values
        template <typename T>
        static T Get(const std::string &key)
        {
            return T(Config::config_->file_[key]);
        }
    };
} // namespace tedvslam

#endif // CONFIG_H
