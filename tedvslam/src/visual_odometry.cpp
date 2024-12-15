
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
        if (Config::SetParameterFile(config_file_path_) == false)
        {
            return false;
        }

        dataset_ =
            Dataset::Ptr(new Dataset(Config::Get<std::string>("dataset_dir")));
        // CHECK_EQ(dataset_->Init(), true);
        assert(dataset_->Init() == true);

        // create components and links
        frontend_ = Frontend::Ptr(new Frontend);
        backend_ = Backend::Ptr(new Backend);
        map_ = Map::Ptr(new Map);
        viewer_ = Viewer::Ptr(new Viewer);

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
        while (1)
        {
            spdlog::info("Run loop {}", i);
            // LOG(INFO) << "VO is running";
            if (Step() == false)
            {
                break;
            }

            if (!pangolin::ShouldQuit())
            {
                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
                vis_display.Activate(vis_camera);

                // std::unique_lock<std::mutex> lock(viewer_data_mutex_);
                if (viewer_->getCurrentFrame())
                {
                    DrawFrame(viewer_->getCurrentFrame(), green);

                    SE3 Twc = viewer_->getCurrentFrame()->Pose().inverse();
                    pangolin::OpenGlMatrix m(Twc.matrix());
                    vis_camera.Follow(m, true);
                    // FollowCurrentFrame(vis_camera_);

                    spdlog::info("Plot Frame Image");
                    cv::Mat img;
                    cv::cvtColor(viewer_->getCurrentFrame()->left_img_, img, cv::COLOR_GRAY2BGR);
                    for (size_t i = 0; i < viewer_->getCurrentFrame()->features_left_.size(); ++i)
                    {
                        if (viewer_->getCurrentFrame()->features_left_[i]->map_point_.lock())
                        {
                            auto feat = viewer_->getCurrentFrame()->features_left_[i];
                            cv::circle(img, feat->position_.pt, 2, cv::Scalar(0, 250, 0),
                                       2);
                        }
                    }
                    spdlog::info("End plot Frame Image");

                    // cv::Mat img = PlotFrameImage();
                    // You cannot use imshow and waitkey in another thread!!!!
                    cv::imshow("image", img);
                    cv::waitKey(1);
                }

                spdlog::info("End Showing Frame");

                if (viewer_->getMap())
                {
                    // DrawMapPoints();

                    for (auto &kf : viewer_->active_keyframes)
                    {
                        DrawFrame(kf.second, red);
                    }

                    glPointSize(2);
                    glBegin(GL_POINTS);
                    for (auto &landmark : viewer_->active_landmarks)
                    {
                        auto pos = landmark.second->Pos();
                        glColor3f(red[0], red[1], red[2]);
                        glVertex3d(pos[0], pos[1], pos[2]);
                    }
                    glEnd();
                }
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

    bool VisualOdometry::Step()
    {
        Frame::Ptr new_frame = dataset_->NextFrame();
        if (new_frame == nullptr)
            return false;

        auto t1 = std::chrono::steady_clock::now();
        bool success = frontend_->AddFrame(new_frame);
        auto t2 = std::chrono::steady_clock::now();
        auto time_used =
            std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        // spdlog::info("VO cost time: {} seconds.", time_used.count());
        // LOG(INFO) << "VO cost time: " << time_used.count() << " seconds.";
        return success;
    }

    void VisualOdometry::DrawFrame(Frame::Ptr frame, const float *color)
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

} // namespace myslam
