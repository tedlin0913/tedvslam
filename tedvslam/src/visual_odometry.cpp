
#include "tedvslam/visual_odometry.h"
#include <chrono>
#include "tedvslam/config.h"
#include "tedvslam/feature.h"
#include <pangolin/pangolin.h>
#include <opencv2/opencv.hpp>
namespace tedvslam
{

    VisualOdometry::VisualOdometry(std::string &config_path)
        : config_file_path_(config_path) {}

    bool VisualOdometry::Init()
    {
        // read from config file
        spdlog::info("Read from config file");
        if (Config::LoadFromFile(config_file_path_) == false)
        {
            return false;
        }

        dataset_ =
            Dataset::Ptr(new Dataset(Config::Get<std::string>("dataset_dir").value_or("")));
        // CHECK_EQ(dataset_->Init(), true);
        assert(dataset_->Init() == true);

        // create components and links
        frontend_ = Frontend::Ptr(new Frontend);
        backend_ = Backend::Ptr(new Backend);
        map_ = Map::Ptr(new Map);
        viewer_ = Viewer::Ptr(new Viewer);

        // frontend, backend, viewer 共用一個地圖
        // viewer 更像是一個queue
        frontend_->SetBackend(backend_);
        frontend_->SetMap(map_);
        frontend_->SetViewer(viewer_);
        frontend_->SetCameras(dataset_->GetCamera(0), dataset_->GetCamera(1));

        backend_->SetMap(map_);
        backend_->SetCameras(dataset_->GetCamera(0), dataset_->GetCamera(1));

        viewer_->SetMap(map_);

        return true;
    }

    void VisualOdometry::Run()
    {
        int i = 0;
        spdlog::info("Start RUN");
        pangolin::CreateWindowAndBind("TedVSLAM", 1024, 768);
        glEnable(GL_DEPTH_TEST);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        pangolin::OpenGlRenderState vis_camera(
            pangolin::ProjectionMatrix(1024, 768, 400, 400, 512, 384, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -5, -10, 0, 0, 0, 0.0, -1.0, 0.0));

        // Add named OpenGL viewport to window and provide 3D Handler
        pangolin::View &vis_display =
            pangolin::CreateDisplay()
                .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
                .SetHandler(new pangolin::Handler3D(vis_camera));

        const float blue[3] = {0, 0, 1};
        const float green[3] = {0, 1, 0};
        const float red[3] = {1.0, 0, 0};
        std::this_thread::sleep_for(std::chrono::microseconds(1000000));
        while (1)
        {
            // spdlog::info("Run loop {}", i);
            // LOG(INFO) << "VO is running";
            if (Step() == false)
            {
                spdlog::error("STEP FALSE");
                break;
            }

            if (!pangolin::ShouldQuit())
            {
                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
                vis_display.Activate(vis_camera);

                // std::unique_lock<std::mutex> lock(viewer_->viewer_data_mutex_);
                if (viewer_->getCurrentFrame())
                {

                    SE3 Twc = viewer_->getCurrentFrame()->Pose().inverse();
                    // Follow camera
                    pangolin::OpenGlMatrix m(Twc.matrix());
                    vis_camera.Follow(m, true);
                    FollowCurrentFrame(vis_camera);

                    spdlog::info("Plot Frame Image");
                    cv::Mat img;
                    cv::cvtColor(viewer_->getCurrentFrame()->left_img_, img, cv::COLOR_GRAY2BGR);
                    for (size_t i = 0; i < viewer_->getCurrentFrame()->features_left_.size(); ++i)
                    {
                        if (viewer_->getCurrentFrame()->features_left_[i]->map_point_.lock())
                        {
                            auto feat = viewer_->getCurrentFrame()->features_left_[i];
                            cv::circle(img, feat->keypoint_.pt, 2, cv::Scalar(0, 250, 0),
                                       2);
                        }
                    }
                    spdlog::info("End plot Frame Image");
                    cv::imshow("image", img);
                    cv::waitKey(50);

                    DrawFrame(viewer_->getCurrentFrame(), green);

                    if (viewer_->getMap())
                    {
                        // DrawMapPoints();

                        for (auto &kf : map_->GetAllKeyFrames())
                        {
                            DrawFrame(kf.second, blue);
                        }

                        glPointSize(2);
                        glBegin(GL_POINTS);
                        for (auto &landmark : map_->GetAllMapPoints())
                        {
                            auto pos = landmark.second->Pos();
                            glColor3f(red[0], red[1], red[2]);
                            glVertex3d(pos[0], pos[1], pos[2]);
                        }
                        glEnd();
                    }

                    // cv::Mat img = PlotFrameImage();
                    // You cannot use imshow and waitkey in another thread!!!!
                }

                spdlog::info("End Showing Frame");

                spdlog::info("Start Finish Frame");
                // WSL2 cannot using pangolin correctly???
                pangolin::FinishFrame();
                std::this_thread::sleep_for(std::chrono::microseconds(5000));
                // std::this_thread::sleep_for(std::chrono::seconds(1));
                spdlog::info("Finish Show");
            }
            i++;
        }

        backend_->Stop();
        viewer_->Close();

        // LOG(INFO) << "VO exit";
    }

    cv::Mat VisualOdometry::PlotFrameImage()
    {
        spdlog::info("Plot Frame Image");
        cv::Mat img_out;
        cv::cvtColor(viewer_->getCurrentFrame()->left_img_, img_out, cv::COLOR_GRAY2BGR);
        for (size_t i = 0; i < viewer_->getCurrentFrame()->features_left_.size(); ++i)
        {
            if (viewer_->getCurrentFrame()->features_left_[i]->map_point_.lock())
            {
                auto feat = viewer_->getCurrentFrame()->features_left_[i];
                cv::circle(img_out, feat->keypoint_.pt, 2, cv::Scalar(0, 250, 0),
                           2);
            }
        }
        spdlog::info("End plot Frame Image");
        return img_out;
    }

    void VisualOdometry::FollowCurrentFrame(pangolin::OpenGlRenderState &vis_camera)
    {
        SE3 Twc = viewer_->getCurrentFrame()->Pose().inverse();
        pangolin::OpenGlMatrix m(Twc.matrix());
        vis_camera.Follow(m, true);
    }

    bool VisualOdometry::Step()
    {
        Frame::Ptr new_frame = dataset_->NextFrame();

        std::this_thread::sleep_for(std::chrono::microseconds(3000));

        if (new_frame == nullptr)
            return false;

        auto t1 = std::chrono::steady_clock::now();
        bool success = frontend_->AddFrame(new_frame);
        auto t2 = std::chrono::steady_clock::now();
        auto time_used =
            std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        spdlog::info("VO cost time: {} seconds.", time_used.count());
        // 300ms = 10fps
        std::this_thread::sleep_for(std::chrono::microseconds(30000));
        // LOG(INFO) << "VO cost time: " << time_used.count() << " seconds.";
        return success;
    }

    void VisualOdometry::DrawFrame(Frame::Ptr frame, const float *color)
    {
        // Draw camera frame with color
        SE3 Twc = frame->Pose().inverse();
        const float sz = 1.0;
        const int line_width = 2.0;
        const float fx = 400;
        const float fy = 400;
        const float cx = 512;
        const float cy = 384;
        const float width = 1080;
        const float height = 768;

        glPushMatrix();

        Sophus::Matrix4f m = Twc.matrix().template cast<float>();
        glMultMatrixf((GLfloat *)m.data());

        if (color == nullptr)
        {
            glColor3f(1, 0, 0);
        }
        else
            glColor3f(color[0], color[1], color[2]);

        glLineWidth(line_width);
        glBegin(GL_LINES);
        glVertex3f(0, 0, 0);
        glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
        glVertex3f(0, 0, 0);
        glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
        glVertex3f(0, 0, 0);
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
        glVertex3f(0, 0, 0);
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

        glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

        glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
        glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

        glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
        glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);

        glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

        glEnd();
        glPopMatrix();
    }

    void VisualOdometry::DrawFrame(KeyFrame::Ptr frame, const float *color)
    {
        SE3 Twc = frame->Pose().inverse();
        const float sz = 1.0;
        const int line_width = 2.0;
        const float fx = 400;
        const float fy = 400;
        const float cx = 512;
        const float cy = 384;
        const float width = 1080;
        const float height = 768;

        glPushMatrix();

        Sophus::Matrix4f m = Twc.matrix().template cast<float>();
        glMultMatrixf((GLfloat *)m.data());

        if (color == nullptr)
        {
            glColor3f(1, 0, 0);
        }
        else
            glColor3f(color[0], color[1], color[2]);

        glLineWidth(line_width);
        glBegin(GL_LINES);
        glVertex3f(0, 0, 0);
        glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
        glVertex3f(0, 0, 0);
        glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
        glVertex3f(0, 0, 0);
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
        glVertex3f(0, 0, 0);
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

        glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

        glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
        glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

        glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
        glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);

        glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
        glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

        glEnd();
        glPopMatrix();
    }

    // void VisualOdometry::DrawMapPoints()
    // {
    //     // Draw the active frames
    //     spdlog::info("Start Draw Map points");
    //     const float red[3] = {1.0, 0, 0};
    //     for (auto &kf : viewer_->active_keyframes)
    //     {
    //         DrawFrame(kf.second, red);
    //     }

    //     // Draw map points
    //     glPointSize(5);
    //     glBegin(GL_POINTS);
    //     for (auto &landmark : viewer_->active_landmarks)
    //     {
    //         auto pos = landmark.second->Pos();
    //         glColor3f(red[0], red[1], red[2]);
    //         glVertex3d(pos[0], pos[1], pos[2]);
    //     }
    //     glEnd();
    //     spdlog::info("End Draw Map points");
    // }

} // namespace myslam
