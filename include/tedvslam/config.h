#pragma once
#ifndef CONFIG_H
#define CONFIG_H

// #include "tedvslam/common_include.h"
#include <yaml-cpp/yaml.h>
#include <optional>
#include <memory>
#include <string>
#include <mutex>
#include <spdlog/spdlog.h>

/**
 * @file Config.h
 * @brief Singleton class for managing application configuration.
 *
 * Usage Example:
 *
 * ```cpp
 * #include "Config.h"
 *
 * int main()
 * {
 *     using namespace tedvslam;
 *
 *     // Load configuration
 *     if (!Config::LoadFromFile("config.yaml"))
 *     {
 *         spdlog::error("Failed to load configuration file.");
 *         return EXIT_FAILURE;
 *     }
 *
 *     // Retrieve parameters
 *     if (auto appName = Config::Get<std::string>("application.name"))
 *     {
 *         spdlog::info("Application Name: {}", *appName);
 *     }
 *     else
 *     {
 *         spdlog::warn("Application name not set.");
 *     }
 *
 *     if (auto appVersion = Config::Get<int>("application.version"))
 *     {
 *         spdlog::info("Application Version: {}", *appVersion);
 *     }
 *
 *     if (auto threshold = Config::Get<double>("settings.threshold"))
 *     {
 *         spdlog::info("Threshold: {}", *threshold);
 *     }
 *
 *     // Check for a key
 *     if (Config::HasKey("settings.max_iterations"))
 *     {
 *         spdlog::info("Max iterations key exists.");
 *     }
 *
 *     return EXIT_SUCCESS;
 * }
 * ```
 */

namespace tedvslam
{
    class Config
    {
    public:
        // Deleted copy constructor and assignment operator to prevent copies
        Config(const Config &) = delete;
        Config &operator=(const Config &) = delete;

        ~Config() = default;

        // Load configuration from file
        static bool LoadFromFile(const std::string &filename);

        // Access configuration values
        template <typename T>
        static std::optional<T> Get(const std::string &key)
        {
            auto instance = InstancePtr();
            std::lock_guard<std::mutex> lock(instance->mutex_);
            if (!instance->file_[key])
            {
                spdlog::warn("Key '{}' not found in configuration.", key);
                return std::nullopt;
            }
            try
            {
                return instance->file_[key].as<T>();
            }
            catch (const YAML::BadConversion &e)
            {
                spdlog::error("Failed to convert key '{}' to requested type: {}", key, e.what());
                return std::nullopt;
            }
        }

        // Check if a key exists
        static bool HasKey(const std::string &key)
        {
            auto instance = InstancePtr();
            std::lock_guard<std::mutex> lock(instance->mutex_);
            return static_cast<bool>(instance->file_[key]);
        }

    private:
        Config() = default;

        static std::shared_ptr<Config> InstancePtr()
        {
            static std::shared_ptr<Config> instance = std::shared_ptr<Config>(new Config());
            return instance;
        }

        YAML::Node file_;
        mutable std::mutex mutex_; // Mutex for thread-safe access
    };
} // namespace tedvslam

#endif // CONFIG_H
