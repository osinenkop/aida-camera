#ifndef __DATA_HXX__
#define __DATA_HXX__

#include <string>
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/types_c.h"

struct Data{

std::string serial_;
std::string port_number_;
std::string index_;

unsigned fps_;
bool is_calibrated_, port_opened_;

cv::Mat camera_matrix_;
cv::Mat distortion_coefficients_;
cv::Mat rectification_matrix_;
cv::Mat projection_matrix_;

std::vector<double> d, k, r, p;
std::string distortion_model_;

std::array<double, 3> initial_pose_;
std::array<double, 3> initial_orientation_;

cv::Mat distorted_frame_, undistorted_frame_;
cv::Mat map1_, map2_;
cv::Size image_size_;

};

#endif