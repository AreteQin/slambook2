//
// Created by gaoxiang on 19-5-4.
//
#include "myslam/visual_odometry.h"
#include <chrono>
#include <memory>
#include "myslam/config.h"

namespace myslam {

    VisualOdometry::VisualOdometry(std::string &config_path)
            : config_file_path_(config_path) {}

    bool VisualOdometry::Init(Camera::Ptr left, Camera::Ptr right, int dataset_type) {
        // read from config file
        if (Config::SetParameterFile(config_file_path_) == false) {
            return false;
        }

//    dataset_ =
//        Dataset::Ptr(new Dataset(Config::Get<std::string>("dataset_dir")));
//    CHECK_EQ(dataset_->Init(), true);

        // create components and links
        frontend_ = std::make_shared<Frontend>();
        backend_ = std::make_shared<Backend>();
        map_ = std::make_shared<Map>();
        viewer_ = std::make_shared<Viewer>();

        frontend_->SetBackend(backend_);
        frontend_->SetMap(map_);
        frontend_->SetViewer(viewer_);
        frontend_->SetCameras(left, right);

        backend_->SetMap(map_);
        backend_->SetCameras(left, right);

        viewer_->SetMap(map_);

        return true;
    }

//void VisualOdometry::Run() {
//    while (1) {
//        LOG(INFO) << "VO is running";
//        if (Step() == false) {
//            break;
//        }
//    }
//
//    backend_->Stop();
//    viewer_->Close();
//
//    LOG(INFO) << "VO exit";
//}

    bool VisualOdometry::Step(Frame::Ptr next_frame) {
        Frame::Ptr new_frame = next_frame;
        if (new_frame == nullptr) return false;

        auto t1 = std::chrono::steady_clock::now();
        bool success = frontend_->AddFrame(new_frame);
        auto t2 = std::chrono::steady_clock::now();
        auto time_used =
                std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
        LOG(INFO) << "VO cost time: " << time_used.count() << " seconds.";
        return success;
    }

}  // namespace myslam
