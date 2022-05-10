//
// Created by qin on 3/9/22.
//

#include "myslam/eth3d_dataset.h"
#include "myslam/frame.h"

#include <boost/format.hpp>
#include <fstream>
#include <opencv2/opencv.hpp>

using namespace std;

//void tokenize(std::string const &str, const char delim,
//              std::vector<std::string> &out) {
//    size_t start;
//    size_t end = 0;
//
//    while ((start = str.find_first_not_of(delim, end)) != std::string::npos) {
//        end = str.find(delim, start);
//        out.push_back(str.substr(start, end - start));
//    }
//}

namespace myslam {

Dataset::Dataset(const std::string &dataset_path)
    : dataset_path_(dataset_path) {}

bool Dataset::Init() {
  // read 1st camera intrinsics
  std::ifstream intrinsic_left(dataset_path_ + "/calibration2.txt");
  if (!intrinsic_left) {
    LOG(ERROR) << "cannot find " << dataset_path_ << "/calibration2.txt";
    return false;
  }
  double K1[4];
  for (double &i : K1) {
    intrinsic_left >> i;
  }
  Vec3 t1;
  t1 << 0, 0, 0;
  Camera::Ptr camera_left(
      new Camera(K1[0], K1[1], K1[2], K1[3], t1.norm(), SE3(SO3(), t1)));
  cameras_.push_back(camera_left);

  // read 2nd camera intrinsics and extrinsics
  std::ifstream intrinsic_right(dataset_path_ + "/calibration.txt");
  if (!intrinsic_right) {
    LOG(ERROR) << "cannot find " << dataset_path_ << "/calibration.txt";
    return false;
  }
  double K2[4];
  for (double &i : K2) {
    intrinsic_right >> i;
  }
  std::ifstream extrinsic_left2right(dataset_path_ + "/extrinsics_1_2.txt");
  if (!extrinsic_left2right) {
    LOG(ERROR) << "cannot find " << dataset_path_ << "/extrinsics_1_2.txt";
    return false;
  }
  double projection_data[12]; // T from right to left camera
  for (double &i : projection_data) {
    extrinsic_left2right >> i;
  }
  Mat33 R2;
  R2 << projection_data[0], projection_data[1], projection_data[2],
      projection_data[4], projection_data[5], projection_data[6],
      projection_data[8], projection_data[9], projection_data[10];
  Eigen::JacobiSVD<Eigen::MatrixXd> SO3_R(R2, Eigen::ComputeThinU | Eigen::ComputeThinV);
  R2 = SO3_R.matrixU().inverse() * SO3_R.matrixV().transpose().inverse();
  Mat33 R = R2.inverse();
  Vec3 t2;
  t2 << projection_data[3], projection_data[7], projection_data[11];
  t2 = -1 * (R * t2);
  Camera::Ptr camera_right(
      new Camera(K2[0], K2[1], K2[2], K2[3], t2.norm(), SE3(R, t2)));
  cameras_.push_back(camera_right);

//        LOG(INFO) << "Camera extrinsics: " << t.transpose();
  extrinsic_left2right.close();
  intrinsic_left.close();
  intrinsic_right.close();

  std::cout << "left K:\n" << camera_left->K().matrix() << std::endl;
  std::cout << "left t:\n" << camera_left->pose().matrix() << std::endl;
  std::cout << "right K:\n" << camera_right->K().matrix() << std::endl;
  std::cout << "right t:\n" << camera_right->pose().matrix() << std::endl;

  // read image pairs
  std::ifstream dataset(dataset_path_ + "/associated.txt");
  if (!dataset) {
    LOG(ERROR) << "cannot find " << dataset_path_ << "/associated.txt";
    return false;
  }
  std::string line;
  while (getline(dataset, line)) {
    char *cstr = new char[line.length() + 1];
    std::strcpy(cstr, line.c_str());
    char *p = std::strtok(cstr + 15, " ");
//    for (int i = 1; i < 2; i++){
//      p = std::strtok(nullptr, " ");
//    }
    std::string each_in_line = p;
    left_images_.push_back(dataset_path_ + "/" + each_in_line.insert(3, "2"));
    right_images_.push_back(dataset_path_ + "/" + each_in_line);
    LOG(INFO) << (dataset_path_ + "/" + each_in_line);
  }

  current_image_index_ = 0;
  return true;

}

Frame::Ptr Dataset::NextFrame() {
  cv::Mat image_left, image_right, image_right_original;
  // read images
  image_left = cv::imread(left_images_[current_image_index_], cv::IMREAD_GRAYSCALE);
  image_right_original = cv::imread(right_images_[current_image_index_], cv::IMREAD_GRAYSCALE);

  cv::resize(image_right_original, image_right, cv::Size(image_left.cols,
                                                         image_left.rows), cv::INTER_LINEAR);

  if (image_left.data == nullptr || image_right.data == nullptr) {
    LOG(WARNING) << "cannot find images at index " << current_image_index_;
    return nullptr;
  }

//        cv::Mat image_left_resized, image_right_resized;
//        cv::resize(image_left, image_left_resized, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);
//        cv::resize(image_right, image_right_resized, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);

  auto new_frame = Frame::CreateFrame();
//        new_frame->left_img_ = image_left_resized;
//        new_frame->right_img_ = image_right_resized;
  new_frame->left_img_ = image_left;
  new_frame->right_img_ = image_right;
  current_image_index_++;
  return new_frame;
}

}  // namespace myslam