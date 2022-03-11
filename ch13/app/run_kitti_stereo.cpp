//
// Created by gaoxiang on 19-5-4.
//

#include <gflags/gflags.h>
#include "myslam/visual_odometry.h"
#include "myslam/kitti_dataset.h"

DEFINE_string(config_file, "./config/default.yaml", "config file path");

int main(int argc, char **argv) {
    google::ParseCommandLineFlags(&argc, &argv, true);

    std::shared_ptr<myslam::Dataset> dataset(
            new myslam::Dataset("/home/qin/Downloads/data_odometry_gray/dataset/sequences/02"));
    CHECK_EQ(dataset->Init(), true);

    myslam::VisualOdometry::Ptr vo(new myslam::VisualOdometry(FLAGS_config_file));
    assert(vo->Init(dataset->GetCamera(0), dataset->GetCamera(1), 1) == true);
//    vo->Run();

    while (true) {
        LOG(INFO) << "VO is running";
        if (!vo->Step(dataset->NextFrame())) {
            break;
        }
    }

//    backend_->Stop();
//    viewer_->Close();

    LOG(INFO) << "VO exit";

    return 0;
}
