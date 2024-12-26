#include "tedvslam/config.h"

namespace tedvslam
{
    bool Config::LoadFromFile(const std::string &filename)
    {
        auto instance = InstancePtr();
        std::lock_guard<std::mutex> lock(instance->mutex_);
        try
        {
            instance->file_ = YAML::LoadFile(filename);
            spdlog::info("Loaded configuration file: {}", filename);
            return true;
        }
        catch (const YAML::Exception &e)
        {
            spdlog::error("Error reading configuration file '{}': {}", filename, e.what());
            return false;
        }
    }
}
