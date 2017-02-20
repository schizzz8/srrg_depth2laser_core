#pragma once
#include <opencv2/core/core.hpp>
#include <string>
#include "srrg_txt_io/laser_message.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace srrg_depth2laser_core{

  class ScanExtractor{
  public:
    ScanExtractor(std::string filename);
    void setParameters(int seq,double timestamp,srrg_core::LaserMessage& laser_msg);
    void compute(const cv::Mat& image,srrg_core::LaserMessage& laser_msg);
  protected:
    std::string _topic_name;
    std::string _frame_id;
    int _num_ranges;
    float _min_range;
    float _max_range;
    float _min_angle;
    float _max_angle;
    float _angle_increment;
    float _inverse_angle_increment;
    float _laser_plane_thickness;
    float _squared_max_norm;
    float _squared_min_norm;
    Eigen::Matrix3f _K;
    Eigen::Matrix3f _invK;
    Eigen::Isometry3f _camera_transform;
    Eigen::Isometry3f _laser_transform;
    Eigen::Isometry3f camera2laser_transform;

  };
}
