//
// Created by gaoxiang on 19-5-4.
//

#include <gflags/gflags.h>
#include "tedvslam/visual_odometry.h"
// #include <glog/logging.h>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h> // For logging to files

DEFINE_string(config_file, "/home/ted/dev/tedvslam/tedvslam/config/default.yaml", "config file path");

int main(int argc, char **argv)
{
    google::ParseCommandLineFlags(&argc, &argv, true);
    // spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%l] [%s:%#] %v");
    spdlog::info("Start demo");
    spdlog::warn("Develop mode");

    // Log with formatting
    // spdlog::error("An error occurred: code = {}", 404);

    // File logger
    // auto file_logger = spdlog::basic_logger_mt("file_logger", "logs/my_log.txt");
    // file_logger->info("This is logged to a file!");

    tedvslam::VisualOdometry::Ptr vo(
        new tedvslam::VisualOdometry(FLAGS_config_file));
    assert(vo->Init() == true);
    vo->Run();

    return 0;
}
