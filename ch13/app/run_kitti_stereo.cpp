//
// Created by gaoxiang on 19-5-4.
//

#include <gflags/gflags.h>
#include "myslam/visual_odometry.h"
#include "myslam/kitti_dataset.h"
#include "myslam/config.h"

DEFINE_string(config_file, "./config/default_kitti.yaml", "config file path");

int main(int argc, char **argv) {
    google::ParseCommandLineFlags(&argc, &argv, true);

    myslam::Config::SetParameterFile(FLAGS_config_file);
    std::shared_ptr<myslam::Dataset> dataset(
            new myslam::Dataset(myslam::Config::Get<std::string>("dataset_dir")));
    CHECK_EQ(dataset->Init(), true);

    myslam::VisualOdometry::Ptr vo(new myslam::VisualOdometry(FLAGS_config_file));
    assert(vo->Init(dataset->GetCamera(0), dataset->GetCamera(1), 1) == true);
//    vo->Run();

    while (true) {
        LOG(INFO) << "VO is running";
        if (!vo->Step(dataset->NextFrame())) {
            break;
        }
        sleep(myslam::Config::Get<int>("sleep_time"));
    }

//    backend_->Stop();
//    viewer_->Close();

    LOG(INFO) << "VO exit";

    return 0;
}
