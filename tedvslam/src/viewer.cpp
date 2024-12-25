#include "tedvslam/viewer.h"
#include "tedvslam/feature.h"
#include "tedvslam/frame.h"

#include <pangolin/pangolin.h>
#include <opencv2/opencv.hpp>

namespace tedvslam
{
    // Opencv and pangolin related functions should run
    // in the main thread. others could run in separate thread.
    // RUN THIS CLASS IN THE MAIN THREAD
    // 把visual odometry裡面的東西移回來！！
    Viewer::Viewer()
    {
    }

    void Viewer::Close()
    {

        // viewer_running_ = false;

        // viewer_thread_.join();
    }

    // FRONTEND => VIEWER
    void Viewer::AddCurrentFrame(Frame::Ptr current_frame)
    {
        std::unique_lock<std::mutex> lck(viewer_data_mutex_);
        current_frame_ = current_frame;
    }

    void Viewer::UpdateMap()
    {
        spdlog::info("Update map");
        std::unique_lock<std::mutex> lck(viewer_data_mutex_);
        assert(map_ != nullptr);
        active_keyframes = map_->GetActiveKeyFrames();
        active_landmarks = map_->GetActiveMapPoints();
        map_updated_ = true;
    }

    // void Viewer::Show()
    // {
    //     int i = 0;
    //     spdlog::info("Start RUN");
    //     pangolin::CreateWindowAndBind("TedVSLAM", 1024, 768);
    //     glEnable(GL_DEPTH_TEST);
    //     glEnable(GL_BLEND);
    //     glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    //     pangolin::OpenGlRenderState vis_camera(
    //         pangolin::ProjectionMatrix(1024, 768, 400, 400, 512, 384, 0.1, 1000),
    //         pangolin::ModelViewLookAt(0, -5, -10, 0, 0, 0, 0.0, -1.0, 0.0));

    //     // Add named OpenGL viewport to window and provide 3D Handler
    //     pangolin::View &vis_display =
    //         pangolin::CreateDisplay()
    //             .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
    //             .SetHandler(new pangolin::Handler3D(vis_camera));

    //     const float blue[3] = {0, 0, 1};
    //     const float green[3] = {0, 1, 0};
    //     const float red[3] = {1.0, 0, 0};

    //     while (!pangolin::ShouldQuit())
    //     {
    //         glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    //         glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    //         vis_display.Activate(vis_camera);

    //         std::unique_lock<std::mutex> lock(viewer_data_mutex_);
    //         if (viewer_->getCurrentFrame())
    //         {
    //             DrawFrame(viewer_->getCurrentFrame(), green);

    //             SE3 Twc = viewer_->getCurrentFrame()->Pose().inverse();
    //             pangolin::OpenGlMatrix m(Twc.matrix());
    //             vis_camera.Follow(m, true);
    //             FollowCurrentFrame(vis_camera);

    //             spdlog::info("Plot Frame Image");
    //             cv::Mat img;
    //             cv::cvtColor(viewer_->getCurrentFrame()->left_img_, img, cv::COLOR_GRAY2BGR);
    //             for (size_t i = 0; i < viewer_->getCurrentFrame()->features_left_.size(); ++i)
    //             {
    //                 if (viewer_->getCurrentFrame()->features_left_[i]->map_point_.lock())
    //                 {
    //                     auto feat = viewer_->getCurrentFrame()->features_left_[i];
    //                     cv::circle(img, feat->position_.pt, 2, cv::Scalar(0, 250, 0),
    //                                2);
    //                 }
    //             }
    //             spdlog::info("End plot Frame Image");

    //             if (viewer_->getMap())
    //             {
    //                 DrawMapPoints();

    //                 for (auto &kf : viewer_->active_keyframes)
    //                 {
    //                     DrawFrame(kf.second, red);
    //                 }

    //                 glPointSize(2);
    //                 glBegin(GL_POINTS);
    //                 for (auto &landmark : viewer_->active_landmarks)
    //                 {
    //                     auto pos = landmark.second->Pos();
    //                     glColor3f(red[0], red[1], red[2]);
    //                     glVertex3d(pos[0], pos[1], pos[2]);
    //                 }
    //                 glEnd();
    //             }

    //             // cv::Mat img = PlotFrameImage();
    //             // You cannot use imshow and waitkey in another thread!!!!
    //             cv::imshow("image", img);
    //             cv::waitKey(10);
    //         }

    //         spdlog::info("End Showing Frame");

    //         spdlog::info("Start Finish Frame");
    //         // WSL2 cannot using pangolin correctly???
    //         pangolin::FinishFrame();
    //         std::this_thread::sleep_for(std::chrono::microseconds(5000));
    //         // std::this_thread::sleep_for(std::chrono::seconds(1));
    //         spdlog::info("Finish Show");
    //     }
    //     i++;
    // }

    // cv::Mat Viewer::PlotFrameImage()
    // {
    //     spdlog::info("Plot Frame Image");
    //     cv::Mat img_out;
    //     cv::cvtColor(current_frame_->left_img_, img_out, cv::COLOR_GRAY2BGR);
    //     for (size_t i = 0; i < current_frame_->features_left_.size(); ++i)
    //     {
    //         if (current_frame_->features_left_[i]->map_point_.lock())
    //         {
    //             auto feat = current_frame_->features_left_[i];
    //             cv::circle(img_out, feat->position_.pt, 2, cv::Scalar(0, 250, 0),
    //                        2);
    //         }
    //     }
    //     spdlog::info("End plot Frame Image");
    //     return img_out;
    // }

    // void Viewer::FollowCurrentFrame(pangolin::OpenGlRenderState &vis_camera)
    // {
    //     SE3 Twc = current_frame_->Pose().inverse();
    //     pangolin::OpenGlMatrix m(Twc.matrix());
    //     vis_camera.Follow(m, true);
    // }

    // void Viewer::DrawFrame(Frame::Ptr frame, const float *color)
    // {
    //     SE3 Twc = frame->Pose().inverse();
    //     const float sz = 1.0;
    //     const int line_width = 2.0;
    //     const float fx = 400;
    //     const float fy = 400;
    //     const float cx = 512;
    //     const float cy = 384;
    //     const float width = 1080;
    //     const float height = 768;

    //     glPushMatrix();

    //     Sophus::Matrix4f m = Twc.matrix().template cast<float>();
    //     glMultMatrixf((GLfloat *)m.data());

    //     if (color == nullptr)
    //     {
    //         glColor3f(1, 0, 0);
    //     }
    //     else
    //         glColor3f(color[0], color[1], color[2]);

    //     glLineWidth(line_width);
    //     glBegin(GL_LINES);
    //     glVertex3f(0, 0, 0);
    //     glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
    //     glVertex3f(0, 0, 0);
    //     glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    //     glVertex3f(0, 0, 0);
    //     glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    //     glVertex3f(0, 0, 0);
    //     glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

    //     glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
    //     glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

    //     glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    //     glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

    //     glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    //     glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);

    //     glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
    //     glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

    //     glEnd();
    //     glPopMatrix();
    // }

    // void Viewer::DrawMapPoints()
    // {
    //     spdlog::info("Start Draw Map points");
    //     // const float red[3] = {1.0, 0, 0};
    //     // for (auto &kf : active_keyframes_)
    //     // {
    //     //     DrawFrame(kf.second, red);
    //     // }

    //     // glPointSize(2);
    //     // glBegin(GL_POINTS);
    //     // for (auto &landmark : active_landmarks_)
    //     // {
    //     //     auto pos = landmark.second->Pos();
    //     //     glColor3f(red[0], red[1], red[2]);
    //     //     glVertex3d(pos[0], pos[1], pos[2]);
    //     // }
    //     // glEnd();
    //     // spdlog::info("End Draw Map points");
    // }

} // namespace myslam
