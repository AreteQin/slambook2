//
// Created by qin on 3/9/22.
//

#include <gflags/gflags.h>
#include "myslam/visual_odometry.h"
#include "myslam/eth3d_dataset.h"
#include "myslam/config.h"

DEFINE_string(config_file, "./config/default_eth3d.yaml", "config file path");

int main(int argc, char **argv) {
    google::ParseCommandLineFlags(&argc, &argv, true);

    myslam::Config::SetParameterFile(FLAGS_config_file);

    std::shared_ptr<myslam::Dataset> dataset(
            new myslam::Dataset(myslam::Config::Get<std::string>("dataset_dir")));
    CHECK_EQ(dataset->Init(), true);

    myslam::VisualOdometry::Ptr vo(new myslam::VisualOdometry(FLAGS_config_file));
    assert(vo->Init(dataset->GetCamera(0), dataset->GetCamera(1), 2) == true);
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