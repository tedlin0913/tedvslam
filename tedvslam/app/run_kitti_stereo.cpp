//
// Created by gaoxiang on 19-5-4.
//

#include <gflags/gflags.h>
#include "tedvslam/visual_odometry.h"

DEFINE_string(config_file, "./config/default.yaml", "config file path");

int main(int argc, char **argv)
{
    google::ParseCommandLineFlags(&argc, &argv, true);

    tedvslam::VisualOdometry::Ptr vo(
        new tedvslam::VisualOdometry(FLAGS_config_file));
    assert(vo->Init() == true);
    vo->Run();

    return 0;
}
