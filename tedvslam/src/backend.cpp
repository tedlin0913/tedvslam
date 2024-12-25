#include "tedvslam/backend.h"
#include "tedvslam/algorithm.h"
#include "tedvslam/feature.h"
#include "tedvslam/g2o_types.h"

#include "tedvslam/mappoint.h"
#include <map>

namespace tedvslam
{

    Backend::Backend()
    {
        is_running_.store(true);
        // request_pause_.store(true);
        // is_paused_.store(true);

        backend_thread_ = std::thread(std::bind(&Backend::RunBackend, this));
    }

    // void Backend::InsertKeyFrame(const std::shared_ptr<KeyFrame> &keyframe)
    // {
    //     spdlog::info("Insert keyframe backend");
    //     std::unique_lock<std::mutex> lck(new_keyframe_mutex_);
    //     new_keyframes_.push_back(keyframe);

    //     // need active map optimization when there is a new KF inserted
    //     need_optimization_ = true;
    //     spdlog::info("DONE Insert keyframe backend");
    //     UpdateMap();
    // }

    void Backend::UpdateMap()
    {
        // {
        //     // We set some variable that indicates “there is new data”
        //     std::unique_lock<std::mutex> lock(new_keyframe_mutex_);
        //     new_data_available_ = true;
        // }
        std::unique_lock<std::mutex> lock(new_keyframe_mutex_);
        map_update_.notify_one();
    }

    // void Backend::RequestPause()
    // {
    //     request_pause_.store(true);
    // }

    // bool Backend::IsPaused() const
    // {
    //     return (request_pause_.load()) && (is_paused_.load());
    // }

    // void Backend::Resume()
    // {
    //     request_pause_.store(false);
    // }

    void Backend::Stop()
    {
        is_running_.store(false);
        map_update_.notify_one();
        backend_thread_.join();
    }

    void Backend::RunBackend()
    {
        spdlog::info("==== Start BACKEND ====");
        while (is_running_.load())
        {
            std::unique_lock<std::mutex> lock(new_keyframe_mutex_);
            map_update_.wait(lock);
            OptimizeActiveMap();
        }

        // while (is_running_.load())
        // {
        //     // std::unique_lock<std::mutex> lock(new_keyframe_mutex_);

        //     while (CheckNewKeyFrames())
        //     {
        //         spdlog::info("CHECK NEW KEYFRAME");
        //         ProcessNewKeyFrame();
        //     }

        //     // if the loopclosing thread asks backend to pause
        //     // this will make sure that the backend will pause in this position, having processed all new KFs in the list
        //     while (request_pause_.load())
        //     {
        //         is_paused_.store(true);
        //         std::this_thread::sleep_for(std::chrono::microseconds(1000));
        //     }

        //     is_paused_.store(false);

        //     if (!CheckNewKeyFrames() && need_optimization_)
        //     {
        //         OptimizeActiveMap();
        //         need_optimization_ = false; // this will become true when next new KF is inserted
        //     }

        //     std::this_thread::sleep_for(std::chrono::microseconds(1000));

        // // Wait until we either have new data or the system is told to stop
        // map_update_.wait(lock, [&]()
        //                  { return !backend_running_.load() || new_data_available_; });

        // if (!backend_running_.load())
        // {
        //     // If we are stopping, break out of the loop
        //     spdlog::warn("====STOP BACKEND====");
        //     // lock.unlock();
        //     break;
        // }

        // new_data_available_ = false;
        // lock.unlock();
        // // map_update_.wait(lock);

        // /// backend only optimize active keyframe and mappoints
        // Map::KeyframesType active_kfs = map_->GetActiveKeyFrames();
        // Map::LandmarksType active_landmarks = map_->GetActiveMapPoints();
        // spdlog::info("Active keyframes: {}, Active landmarks: {}",
        //              active_kfs.size(), active_landmarks.size());
        // Optimize(active_kfs, active_landmarks);
    }

    // bool Backend::CheckNewKeyFrames()
    // {
    //     std::unique_lock<std::mutex> lck(new_keyframe_mutex_);
    //     return (!new_keyframes_.empty());
    // }

    // void Backend::ProcessNewKeyFrame()
    // {
    //     {
    //         std::unique_lock<std::mutex> lck(new_keyframe_mutex_);
    //         current_keyframe_ = new_keyframes_.front();
    //         new_keyframes_.pop_front();
    //     }

    //     map_->InsertKeyFrame(current_keyframe_);
    // }

    void Backend::OptimizeActiveMap()
    {
        spdlog::info("====OPTIMIZE BACKEND====");

        // Validate map_ pointer
        if (!map_)
        {
            spdlog::error("Map pointer is null. Cannot proceed with optimization.");
            return;
        }

        Map::KeyFramesType keyframes = map_->GetActiveKeyFrames();
        Map::MapPointsType landmarks = map_->GetActiveMapPoints();

        if (keyframes.empty() || landmarks.empty())
        {
            spdlog::warn("No keyframes or landmarks to optimize.");
            return;
        }
        // setup g2o
        typedef g2o::BlockSolver_6_3 BlockSolverType;
        typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType>
            LinearSolverType;
        std::unique_ptr<LinearSolverType> linearSolver(new LinearSolverType);
        std::unique_ptr<BlockSolverType> solver_ptr(new BlockSolverType(std::move(linearSolver)));
        g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));

        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);

        // pose vertex, keyframe id
        std::map<unsigned long, VertexPose *> vertices;
        unsigned long max_kf_id = 0;
        for (auto &keyframe : keyframes)
        {
            auto kf = keyframe.second;

            if (!kf)
            {
                spdlog::warn("Encountered null keyframe pointer with ID {}.", keyframe.first);
                continue;
            }
            VertexPose *vertex_pose = new VertexPose(); // camera vertex_pose
            vertex_pose->setId(kf->keyframe_id_);
            vertex_pose->setEstimate(kf->Pose());
            optimizer.addVertex(vertex_pose);
            if (kf->keyframe_id_ > max_kf_id)
            {
                max_kf_id = kf->keyframe_id_;
            }

            vertices.insert({kf->keyframe_id_, vertex_pose});
        }
        spdlog::info("Added {} vertexes.", vertices.size());
        // mappoint vertex, use id to index
        std::map<unsigned long, VertexXYZ *> vertices_landmarks;
        if (!camera_left_ || !camera_right_)
        {
            spdlog::error("Camera pointers are null. Cannot proceed with optimization.");
            return;
        }

        // camera intrinsics and exterisics
        Mat33 K = camera_left_->K();
        SE3 left_ext = camera_left_->pose();
        SE3 right_ext = camera_right_->pose();

        // edges
        int index = 1;
        double chi2_th = 5.991; // robust kernel threshold
        std::map<EdgeProjection *, Feature::Ptr> edges_and_features;

        for (auto &landmark : landmarks)
        {
            if (!landmark.second)
            {
                spdlog::warn("Encountered null landmark pointer with ID {}.", landmark.first);
                continue;
            }
            if (landmark.second->is_outlier_)
                continue;
            unsigned long landmark_id = landmark.second->id_;
            // spdlog::info("Get obs");
            auto observations = landmark.second->GetObservations();
            for (auto &obs : observations)
            {
                spdlog::info("Obs");
                if (obs.lock() == nullptr)
                    continue;
                auto feat = obs.lock();
                if (!feat)
                {
                    spdlog::warn("Encountered null feature pointer in observations of landmark ID {}.", landmark_id);
                    continue;
                }
                if (feat->is_outlier_ || feat->keyframe_.lock() == nullptr)
                    continue;

                auto frame = feat->keyframe_.lock();
                if (!frame)
                {
                    spdlog::warn("Encountered null keyframe pointer in feature observations.");
                    continue;
                }
                EdgeProjection *edge = nullptr;
                if (feat->is_on_left_frame_)
                {
                    edge = new EdgeProjection(K, left_ext);
                }
                else
                {
                    edge = new EdgeProjection(K, right_ext);
                }

                // and a new vertex if landmark has not been optimized
                if (vertices_landmarks.find(landmark_id) ==
                    vertices_landmarks.end())
                {
                    VertexXYZ *v = new VertexXYZ;
                    v->setEstimate(landmark.second->Pos());
                    v->setId(landmark_id + max_kf_id + 1);
                    v->setMarginalized(true);
                    vertices_landmarks.insert({landmark_id, v});
                    optimizer.addVertex(v);
                }

                if (vertices.find(frame->keyframe_id_) !=
                        vertices.end() &&
                    vertices_landmarks.find(landmark_id) !=
                        vertices_landmarks.end())
                {
                    edge->setId(index);
                    edge->setVertex(0, vertices.at(frame->keyframe_id_));   // pose
                    edge->setVertex(1, vertices_landmarks.at(landmark_id)); // landmark
                    edge->setMeasurement(toVec2(feat->keypoint_.pt));
                    edge->setInformation(Mat22::Identity());
                    auto rk = new g2o::RobustKernelHuber();
                    rk->setDelta(chi2_th);
                    edge->setRobustKernel(rk);
                    edges_and_features.insert({edge, feat});
                    optimizer.addEdge(edge);
                    index++;
                }
                // else
                // {
                //     delete edge;
                // }
            }
        }
        spdlog::info("DONE OBS");
        // do optimization and eliminate the outliers

        int cnt_outlier = 0, cnt_inlier = 0;
        for (int iteration = 0; iteration < 2; ++iteration)
        {
            spdlog::info("START OPTIMIZE");
            optimizer.initializeOptimization();
            optimizer.optimize(10);

            cnt_outlier = 0;
            cnt_inlier = 0;

            // Determine if we want to adjust the outlier threshold
            for (const auto &ef : edges_and_features)
            {
                if (ef.first->chi2() > chi2_th)
                {
                    cnt_outlier++;
                }
                else
                {
                    cnt_inlier++;
                }
            }

            double inlier_ratio = 0.0;
            if ((cnt_inlier + cnt_outlier) > 0)
            {
                inlier_ratio = static_cast<double>(cnt_inlier) / double(cnt_inlier + cnt_outlier);
            }
            else
            {
                inlier_ratio = 0;
            }

            if (inlier_ratio > 0.5)
            {
                break;
            }
            else
            {
                chi2_th *= 2;
            }
        }
        spdlog::info("DONE OPTIMIZE");

        for (auto &ef : edges_and_features)
        {
            if (ef.first->chi2() > chi2_th)
            {
                ef.second->is_outlier_ = true;
                // remove the observation
                // ef.second->map_point_.lock()->RemoveObservation(ef.second);
                // ef.second->map_point_.lock()->RemoveActiveObservation(ef.second);
            }
            else
            {
                ef.second->is_outlier_ = false;
            }
        }
        spdlog::info("DONE EF");
        {
            // std::unique_lock<std::mutex> lck(map_->map_update_mutex_);
            spdlog::info("BACKEND: Out/In {}/{} ", cnt_outlier, cnt_inlier);
            // Set pose and landmark position
            for (auto &v : vertices)
            {
                keyframes.at(v.first)->SetPose(v.second->estimate());
            }
            for (auto &v : vertices_landmarks)
            {
                landmarks.at(v.first)->SetPos(v.second->estimate());
            }

            map_->RemoveAllOutlierMapPoints();
            map_->RemoveOldActiveMapPoints();
        }
    }

} // namespace tedvslam