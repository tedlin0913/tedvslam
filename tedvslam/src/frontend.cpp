
#include "tedvslam/frontend.h"

namespace tedvslam
{

    Frontend::Frontend()
    {

        num_features_init_good_ = Config::Get<int>("numFeatures.initGood").value_or(0);
        num_features_tracking_good_ = Config::Get<int>("numFeatures.trackingGood").value_or(0);
        num_features_tracking_bad_ = Config::Get<int>("numFeatures.trackingBad").value_or(0);

        int num_init_features = Config::Get<int>("ORBextractor.nInitFeatures").value_or(0);
        float scale_factor = Config::Get<float>("ORBextractor.scaleFactor").value_or(1.6);
        int num_levels = Config::Get<int>("ORBextractor.nLevels").value_or(12);
        int ini_fast_thr = Config::Get<int>("ORBextractor.iniThFAST").value_or(0);
        int min_fast_thr = Config::Get<int>("ORBextractor.minThFAST").value_or(0);

        need_undistortion_ = Config::Get<int>("Camera.bNeedUndistortion").value_or(1);

        std::vector<float> coeffs0 = Config::Get<std::vector<float>>("Camera0.DistortionCoeffs").value_or(std::vector<float>{0.0, 0.0, 0.0, 0.0});

        if (coeffs0.size() < 4)
        {
            throw std::runtime_error("Invalid number of distortion coefficients");
        }
        dist_coeffs0_ = cv::Mat(coeffs0).reshape(1, 1);

        std::vector<float> coeffs1 = Config::Get<std::vector<float>>("Camera1.DistortionCoeffs").value_or(std::vector<float>{0.0, 0.0, 0.0, 0.0});

        if (coeffs1.size() < 4)
        {
            throw std::runtime_error("Invalid number of distortion coefficients");
        }
        dist_coeffs1_ = cv::Mat(coeffs1).reshape(1, 1);

        spdlog::info("Camera0 Distortion Coefficients: {}", fmt::join(coeffs0, ", "));

        // Log distortion coefficients for Camera1
        spdlog::info("Camera1 Distortion Coefficients: {}", fmt::join(coeffs1, ", "));

        // // TODO: add orbfeatures?
        gftt_ =
            cv::GFTTDetector::create(Config::Get<int>("num_features").value_or(0), 0.01, 20);
        num_features_init_ = Config::Get<int>("num_features_init").value_or(0);

        orb_ = cv::ORB::create(
            Config::Get<int>("num_features").value_or(0), // Number of features
            scale_factor,                                 // Scale factor
            num_levels,                                   // Number of levels
            31,                                           // Edge threshold (default: 31)
            0,                                            // First level (default: 0)
            2,                                            // WTA_K (default: 2)
            cv::ORB::HARRIS_SCORE,                        // Score type (default: HARRIS_SCORE)
            31,                                           // Patch size (default: 31)
            ini_fast_thr                                  // FAST threshold
        );
        relative_motion_ = Sophus::SE3d();
        // num_features_ = Config::Get<int>("num_features");
        // spdlog::info("Features init: {}", num_features_init_);
        // spdlog::info("Features num: {}", num_features_);
    }

    // 前端運作的發起點，從visual odometry呼叫
    bool Frontend::AddFrame(Frame::Ptr frame)
    {
        current_frame_ = frame;

        // Check if the left and right images are successfully loaded
        if (current_frame_->left_img_.empty() || current_frame_->right_img_.empty())
        {
            spdlog::error("Failed to load images: left_img_ or right_img_ is empty.");
            return false;
        }

        if (need_undistortion_)
        {
            spdlog::info("Undistort image");
            camera_left_->undistortImage(current_frame_->left_img_, current_frame_->left_img_, dist_coeffs0_);
            camera_right_->undistortImage(current_frame_->right_img_, current_frame_->right_img_, dist_coeffs1_);
        }

        {

            switch (status_)
            {
            case FrontendStatus::INITING:
                StereoInit();
                break;
            // 好壞都繼續track
            case FrontendStatus::TRACKING_GOOD:
            case FrontendStatus::TRACKING_BAD:
                Track();
                break;
            // For now when it lost will not go back?
            case FrontendStatus::LOST:
                // lost 之後重新init
                Reset();
                // return false;
                break;
            }
        }

        last_frame_ = current_frame_;
        return true;
    }

    bool Frontend::Track()
    {
        // 問題出在第二禎沒有算速度，無法用光流去估計

        // 估計relative pose的原因是找比較接近的pose比較容易收斂
        // 如果有last keyframe就用速度模型估算一下新的相對位置
        // 還有一個問題是沒有運動基本上就沒有辦法估計

        // 原本的作法是用光流追蹤上一禎左邊
        if (last_frame_)
        {
            current_frame_->SetRelativePose(relative_motion_ * last_frame_->RelativePose());
        }
        TrackLastFrame();
        // 但是這在eurco容易跟丟，所以可以重新偵測（反正沒有時間限制）

        // 優化current pose
        tracking_inliers_ = EstimateCurrentPose();
        spdlog::info("TRACKING INLIERS: {}", tracking_inliers_);

        relative_motion_ = current_frame_->RelativePose() * last_frame_->RelativePose().inverse();
        // Three status according to settings.
        if (tracking_inliers_ > num_features_tracking_good_)
        {
            // tracking good
            spdlog::info("====TRACKING GOOD====");
            status_ = FrontendStatus::TRACKING_GOOD;
            // InsertKeyFrame();
        }
        else if (tracking_inliers_ > num_features_tracking_bad_)
        {

            // tracking bad
            spdlog::warn("====TRACKING BAD====");
            status_ = FrontendStatus::TRACKING_BAD;
            DetectFeatures();
            FindFeaturesInRight();
            TriangulateNewPoints();
            InsertKeyFrame();
        }
        else
        {
            // lost
            spdlog::error("====TRACKING LOST====");
            status_ = FrontendStatus::LOST;
        }

        // InsertKeyFrame();

        // if (status_ == FrontendStatus::TRACKING_BAD)
        // {
        // }

        if (viewer_)
        {
            viewer_->AddCurrentFrame(current_frame_);
            viewer_->UpdateMap();
        }

        return true;
    }

    int Frontend::TrackLastFrame()
    {
        // use LK flow to estimate points between the last frame.and the current frame.
        std::vector<cv::Point2f> kps_last, kps_current;
        kps_last.reserve(last_frame_->features_left_.size());
        kps_current.reserve(last_frame_->features_left_.size());

        for (auto &kp : last_frame_->features_left_)
        {
            if (kp->map_point_.lock())
            {
                // use project point
                auto mp = kp->map_point_.lock();
                // if the feature links to a mappoint, use the reprojection result as the initial value
                auto px =
                    camera_left_->world2pixel(mp->Pos(), current_frame_->RelativePose() * reference_kf_->Pose());
                kps_last.emplace_back(kp->keypoint_.pt);
                kps_current.emplace_back(cv::Point2f(px[0], px[1]));
            }
            else
            {
                kps_last.emplace_back(kp->keypoint_.pt);
                kps_current.emplace_back(kp->keypoint_.pt);
            }
        }

        std::vector<uchar> status;
        cv::Mat error;
        cv::calcOpticalFlowPyrLK(
            last_frame_->left_img_, current_frame_->left_img_, kps_last,
            kps_current, status, error, cv::Size(21, 21), 3,
            cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                             0.01),
            cv::OPTFLOW_USE_INITIAL_FLOW);

        int num_good_pts = 0;
        current_frame_->features_left_.reserve(kps_current.size());
        for (size_t i = 0; i < status.size(); ++i)
        {
            if (status[i])
            {
                cv::KeyPoint kp(kps_current[i], 7);
                Feature::Ptr feature = Feature::Create(kp);
                // Feature::Ptr feature(new Feature(current_frame_, kp));
                feature->map_point_ = last_frame_->features_left_[i]->map_point_;
                current_frame_->features_left_.emplace_back(feature);
                num_good_pts++;
            }
        }
        //
        spdlog::info("Find: {} in last image", num_good_pts);
        return num_good_pts;
    }

    // BA 的目標是 3D(triangulation)-2D (特徵在相機平面上的投影)
    // 求解目前Frame的Pose （目前的frame是看左側相機）
    // 右側相機單純用來求特徵3D點不追蹤
    int Frontend::EstimateCurrentPose()
    {
        spdlog::info("====ESTIMATE POSE====");
        // setup g2o
        typedef g2o::BlockSolver_6_3 BlockSolverType;
        typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>
            LinearSolverType;
        std::unique_ptr<LinearSolverType> linearSolver(new LinearSolverType);
        std::unique_ptr<BlockSolverType> solver_ptr(new BlockSolverType(std::move(linearSolver)));
        g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(std::move(solver_ptr));

        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);

        // 相機 Pose 的 vertex
        VertexPose *vertex_pose = new VertexPose(); // camera vertex_pose
        vertex_pose->setId(0);
        // 找一個接近目前點的pose （最近一個kf 的pose），比較容易收斂
        // vertex_pose->setEstimate(current_frame_->Pose());
        vertex_pose->setEstimate(current_frame_->RelativePose() * reference_kf_->Pose());
        optimizer.addVertex(vertex_pose);

        // K 左側相機內參矩陣
        Mat33 K = camera_left_->K();

        // 相機vertex pose到 3D點 vertex pose 之間的 edges
        // 3D 點用 mappoint 紀錄

        // 加入新的Edge
        int index = 1;
        std::vector<EdgeProjectionPoseOnly *> edges;
        std::vector<Feature::Ptr> features;

        spdlog::info("Feature Size: {}", current_frame_->features_left_.size());

        features.reserve(current_frame_->features_left_.size());
        edges.reserve(current_frame_->features_left_.size());
        for (size_t i = 0; i < current_frame_->features_left_.size(); ++i)
        {
            auto mp = current_frame_->features_left_[i]->map_point_.lock();
            if (mp && mp->is_outlier_ == false)
            {
                features.emplace_back(current_frame_->features_left_[i]);
                EdgeProjectionPoseOnly *edge =
                    new EdgeProjectionPoseOnly(mp->Pos(), K);
                edge->setId(index);
                edge->setVertex(0, vertex_pose);
                edge->setMeasurement(
                    toVec2(current_frame_->features_left_[i]->keypoint_.pt));
                edge->setInformation(Eigen::Matrix2d::Identity());
                edge->setRobustKernel(new g2o::RobustKernelHuber);
                edges.emplace_back(edge);
                optimizer.addEdge(edge);
                index++;
            }
        }
        spdlog::info("Edge Number: {}", index);

        // estimate the Pose then determine the outliers
        const double chi2_th = 5.991;
        int cnt_outlier = 0;

        for (int iteration = 0; iteration < 4; ++iteration)
        {

            // if (optimizer.edges().empty())
            // {
            //     spdlog::warn("No edges remain; skipping optimization.");
            //     break;
            // }

            vertex_pose->setEstimate(current_frame_->Pose());
            spdlog::info("# Vertices: {}, # Edges: {}",
                         optimizer.vertices().size(),
                         optimizer.edges().size());

            optimizer.initializeOptimization();
            optimizer.optimize(20);

            cnt_outlier = 0;

            // count the outliers
            for (size_t i = 0; i < edges.size(); ++i)
            {
                auto e = edges[i];
                if (features[i]->is_outlier_)
                {
                    e->computeError();
                }
                if (e->chi2() > chi2_th)
                {
                    features[i]->is_outlier_ = true;
                    e->setLevel(1);
                    cnt_outlier++;
                }
                else
                {
                    features[i]->is_outlier_ = false;
                    e->setLevel(0);
                };

                if (iteration == 2)
                {
                    e->setRobustKernel(nullptr);
                }
            }
        }

        spdlog::info("Outlier/Inlier in pose estimating: {}/{}",
                     cnt_outlier,
                     features.size() - cnt_outlier);

        current_frame_->SetPose(vertex_pose->estimate());
        current_frame_->SetRelativePose(vertex_pose->estimate() * reference_kf_->Pose().inverse());

        // Current frame 的feature和features是共用的shared pointer,
        // 這裡就是把Current frame上面的 outlier 清除掉
        for (auto &feat : features)
        {
            if (feat->is_outlier_)
            {
                feat->map_point_.reset();
                feat->is_outlier_ = false; // maybe we can still use it in future
            }
        }
        return features.size() - cnt_outlier;
    }

    // Observation指的是從這 mappoint(3D) 可以被哪些feature（2D) 看到
    void Frontend::SetObservationsForKeyFrame()
    {
        for (auto &feat : current_frame_->features_left_)
        {
            // waek_pointer.lock(): Creates a new std::shared_ptr that shares ownership of the managed object.
            auto mp = feat->map_point_.lock();
            if (mp)
                mp->AddObservation(feat);
        }
    }

    int Frontend::TriangulateNewPoints()
    {
        // 左右相機位置應由後端更新
        // 後端理論上應該要不斷的更新相機位置
        // 不應該用速度模型去估計點的位置，因為三角化的結果會作為優化的參數
        std::vector<SE3> poses{camera_left_->pose(), camera_right_->pose()};
        // current camera to world pose （left)
        SE3 current_pose_Twc = (current_frame_->Pose() * reference_kf_->Pose()).inverse();
        int cnt_triangulated_pts = 0;
        for (size_t i = 0; i < current_frame_->features_left_.size(); ++i)
        {
            if (current_frame_->features_left_[i]->map_point_.expired() &&
                current_frame_->features_right_[i] != nullptr)
            {
                // triangulation
                // 圖片已經undistort過了所以不需要再用到camera intrinsics去校正點
                std::vector<Vec3> points{
                    camera_left_->pixel2camera(
                        Vec2(current_frame_->features_left_[i]->keypoint_.pt.x,
                             current_frame_->features_left_[i]->keypoint_.pt.y)),
                    camera_right_->pixel2camera(
                        Vec2(current_frame_->features_right_[i]->keypoint_.pt.x,
                             current_frame_->features_right_[i]->keypoint_.pt.y))};
                Vec3 pworld = Vec3::Zero();

                bool tri_ok = triangulation(poses, points, pworld);
                if (!tri_ok)
                {
                    spdlog::warn("Triangulation failed for feature i = {}", i);
                }
                else if (pworld[2] <= 0)
                {
                    spdlog::warn("Negative/zero depth for feature i = {}", i);
                }
                else if (tri_ok && pworld[2] > 0)
                {
                    spdlog::info("Create New Map Point");

                    pworld = current_pose_Twc * pworld;
                    auto new_map_point = MapPoint::Create(pworld);
                    // 鄉對於current pose的座標轉換成世界座標
                    // new_map_point->SetPos(pworld);

                    // 設定新的observation
                    new_map_point->AddObservation(
                        current_frame_->features_left_[i]);
                    new_map_point->AddObservation(
                        current_frame_->features_right_[i]);

                    // 更新feature看到的map point
                    current_frame_->features_left_[i]->map_point_ = new_map_point;
                    current_frame_->features_right_[i]->map_point_ = new_map_point;

                    // 地圖插入新的點
                    map_->InsertMapPoint(new_map_point);
                    cnt_triangulated_pts++;
                }
                else
                {
                    spdlog::error("WTF: {}, {}", tri_ok, pworld[2]);
                }
            }
        }
        // rclcpp::RCLCPP_INFO(rclcpp::get_logger("global_logger") this->get_logger(), "New landmarks: %d", cnt_triangulated_pts);
        spdlog::info("New Landmarks: {} ", cnt_triangulated_pts);
        // LOG(INFO) << "new landmarks: " << cnt_triangulated_pts;
        return cnt_triangulated_pts;
    }

    bool Frontend::StereoInit()
    {
        // TODO: NEED NO return for detect features.
        // 左邊extract feature
        int num_features_left = DetectFeatures();
        // 右邊光流追蹤 feature
        int num_coor_features = FindFeaturesInRight();
        spdlog::info("Init Left: {}, RIGHT: {}", num_features_left, num_coor_features);

        if (num_coor_features < num_features_init_)
        {
            return false;
        }

        // 三角化做出初始地圖
        // FIXME: 初始建圖目前只回傳true所以一定會成功
        // 過濾不成功的條件
        bool build_map_success = BuildInitMap();
        if (build_map_success)
        {
            // Viewer 更新目前的地圖
            status_ = FrontendStatus::TRACKING_GOOD;
            if (viewer_)
            {
                viewer_->AddCurrentFrame(current_frame_);
                viewer_->UpdateMap();
            }
            spdlog::info("Stereo Initialize Success");
            return true;
        }
        spdlog::warn("Stereo Initialize Failed");
        return false;
    }

    // FRONTEND => FEATURE EXTRACTOR
    int Frontend::DetectFeatures()
    {

        // 這邊類似簡單的 NMS，用一個遮罩決定哪裡要做feature extraction，
        // 避免同一個區域一直被檢測導致一個點有多個feature
        // 10, 10 是去除的區域大小單位
        // 去除左側檢測過的位置
        // 比較好的作法還是做NMS
        cv::Mat mask(current_frame_->left_img_.size(), CV_8UC1, 255);
        for (auto &feat : current_frame_->features_left_)
        {
            cv::rectangle(mask, feat->keypoint_.pt - cv::Point2f(5, 5),
                          feat->keypoint_.pt + cv::Point2f(5, 5), 0, cv::FILLED);
        }

        std::vector<cv::KeyPoint> keypoints;
        // gftt_->detect(current_frame_->left_img_, keypoints, mask);
        orb_->detect(current_frame_->left_img_, keypoints, mask);
        int cnt_detected = 0;
        for (auto &kp : keypoints)
        {
            current_frame_->features_left_.push_back(
                Feature::Create(kp));
            cnt_detected++;
        }
        spdlog::info("Detect {} new features", cnt_detected);
        return cnt_detected;
    }

    int Frontend::FindFeaturesInRight()
    {
        // 左側抓的的kp，用光流取得在右側的位置
        // 優點是不用在右邊再做一次feature extraction
        // 不需要特徵匹配、計算描述子
        // 但是要求相機運動比較平滑,很容易受到左右相機光線變化不一樣影響

        if (current_frame_->features_left_.empty())
        {
            spdlog::error("No points in the left image to track. Optical flow calculation skipped.");
            return 0;
        }

        // use LK flow to estimate points in the right image
        std::vector<cv::Point2f> kps_left, kps_right;
        for (auto &kp : current_frame_->features_left_)
        {
            kps_left.push_back(kp->keypoint_.pt);
            auto mp = kp->map_point_.lock();
            if (mp)
            {
                // 如果有成功三角化找到過世界座標，就直接左邊投影到右邊當初始值
                // use projected points as initial guess
                auto px =
                    camera_right_->world2pixel(mp->Pos(), current_frame_->Pose());
                kps_right.push_back(cv::Point2f(px[0], px[1]));
            }
            else
            {
                // 沒有三角化過，則直接使用當初值
                // use same pixel in left iamge
                kps_right.push_back(kp->keypoint_.pt);
            }
        }

        std::vector<uchar> status;
        cv::Mat error;
        cv::calcOpticalFlowPyrLK(
            current_frame_->left_img_, current_frame_->right_img_, kps_left,
            kps_right, status, error, cv::Size(11, 11), 5,
            cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                             0.01),
            cv::OPTFLOW_USE_INITIAL_FLOW);

        int num_good_pts = 0;
        for (size_t i = 0; i < status.size(); ++i)
        {
            if (!status[i])
            {
                spdlog::debug("Right match {} failed optical flow", i);
            }
        }
        for (size_t i = 0; i < status.size(); ++i)
        {
            if (status[i])
            {
                cv::KeyPoint kp(kps_right[i], 7);
                Feature::Ptr feat = Feature::Create(kp);
                feat->is_on_left_frame_ = false;
                current_frame_->features_right_.push_back(feat);
                num_good_pts++;
            }
            else
            {
                current_frame_->features_right_.push_back(nullptr);
            }
        }
        spdlog::info("Find: {} in RIGHT", num_good_pts);
        return num_good_pts;
    }

    bool Frontend::BuildInitMap()
    {
        std::vector<SE3> poses{camera_left_->pose(), camera_right_->pose()};
        size_t cnt_init_landmarks = 0;

        size_t non_null_right = 0;
        // 這裡的current frame是第一偵
        // TODO: check if correct
        SE3 current_pose_Twc = current_frame_->Pose().inverse();

        for (auto &fr : current_frame_->features_right_)
        {
            if (fr != nullptr)
                non_null_right++;
        }
        spdlog::info("Right features total: {}, non-null: {}",
                     current_frame_->features_right_.size(), non_null_right);

        for (size_t i = 0; i < current_frame_->features_left_.size(); ++i)
        {
            if (current_frame_->features_right_[i] == nullptr)
                continue;
            // create map point from triangulation
            std::vector<Vec3> points{
                camera_left_->pixel2camera(
                    Vec2(current_frame_->features_left_[i]->keypoint_.pt.x,
                         current_frame_->features_left_[i]->keypoint_.pt.y)),
                camera_right_->pixel2camera(
                    Vec2(current_frame_->features_right_[i]->keypoint_.pt.x,
                         current_frame_->features_right_[i]->keypoint_.pt.y))};
            Vec3 pworld = Vec3::Zero();
            // spdlog::info("Start Triangulation");
            bool tri_ok = triangulation(poses, points, pworld);
            spdlog::info("{},{},{}", pworld[0], pworld[1], pworld[2]);
            if (!tri_ok)
            {
                spdlog::warn("Triangulation failed for feature i = {}", i);
            }
            else if (pworld[2] <= 0)
            {
                spdlog::warn("Negative/zero depth for feature i = {}", i);
            }
            else if (tri_ok && pworld[2] > 0)
            {
                spdlog::info("Create New Map Point");

                pworld = current_pose_Twc * pworld;
                auto new_map_point = MapPoint::Create(pworld);

                // TODO: not sure if remove or not?
                new_map_point->AddObservation(
                    current_frame_->features_left_[i]);
                new_map_point->AddObservation(
                    current_frame_->features_right_[i]);

                current_frame_->features_left_[i]->map_point_ = new_map_point;
                current_frame_->features_right_[i]->map_point_ = new_map_point;
                map_->InsertMapPoint(new_map_point);
                cnt_init_landmarks++;
            }
            else
            {
                spdlog::error("WTF: {}, {}", tri_ok, pworld[2]);
            }
            // spdlog::info("Done Triangulation");
        }

        spdlog::info("Done Triangulation");

        // 這個時候還沒出發，左邊算是原點，不需要優化位置

        // current_frame_->SetKeyFrame();
        Vec6 se3_zero;
        se3_zero.setZero();
        KeyFrame::Ptr newKF = KeyFrame::Create(current_frame_);
        newKF->SetPose(Sophus::SE3d::exp(se3_zero));
        reference_kf_ = newKF;
        // current_frame_->SetRelativePose(Sophus::SE3d::exp(se3_zero));
        current_frame_->SetRelativePose(Sophus::SE3d::exp(se3_zero));
        SetObservationsForKeyFrame();
        map_->InsertKeyFrame(newKF);
        backend_->UpdateMap();

        spdlog::info("Initial map created with:{} map points", cnt_init_landmarks);

        return true;
    }

    // FRONTEND => BACKEND
    bool Frontend::InsertKeyFrame()
    {

        Vec6 se3_zero;
        se3_zero.setZero();

        KeyFrame::Ptr newKF = KeyFrame::Create(current_frame_);
        if (status_ == FrontendStatus::INITING)
        {
            newKF->SetPose(Sophus::SE3d::exp(se3_zero));
        }
        else
        {
            newKF->SetPose(current_frame_->RelativePose() * reference_kf_->Pose());
            newKF->last_keyframe_ = reference_kf_;
            newKF->relative_pose_to_last_keyframe_ = current_frame_->RelativePose();
        }

        reference_kf_ = newKF;

        current_frame_->SetRelativePose(Sophus::SE3d::exp(se3_zero));

        map_->InsertKeyFrame(newKF);

        SetObservationsForKeyFrame();

        backend_->UpdateMap();

        return true;
    }

    bool Frontend::Reset()
    {
        status_ = FrontendStatus::INITING;
        return true;
    }

} // namespace tedvslam