#ifndef __CAMERA_HXX__
#define __CAMERA_HXX__

#include <algorithm>
#include <string>
#include <unordered_map>
#include <vector>
#include <exception>
#include <array>

// Extras
#include "Data.hxx"

class Camera {
public:
  ~Camera();

  auto setIndex(std::string) -> void;
  auto setSerial(std::string) -> void;
  auto setPortNumber(std::string) -> void;
  auto setResolution(std::string) -> void;
  auto setFPS(unsigned) -> void;
  auto setInitialPose(std::string) -> void;
  auto setInitialOrientation(std::string) -> void;
  auto setCalibrationStatus(bool) -> void;
  auto setCameraMatrix(std::vector<double>) -> void;

  auto setDistortionModel(std::string) -> void;
  auto setDistortionMatrix(std::vector<double>) -> void;
  auto setDistortionCoefficients(std::vector<double>) -> void;
  auto setRectificationMatrix(std::vector<double>) -> void;
  auto setProjectionMatrix(std::vector<double>) -> void;


  // Getter
  auto getFPS() -> double;
  auto getIndex() -> std::string;

  // Extras
  auto splitString(std::string &, char) -> std::vector<std::string>;

  // Operation
  auto warmUp() -> void;
  auto prepareFrame() -> void;
  auto undistortImage() -> void;

  // protected:
  Data data_;
  cv::VideoCapture core_;

};

#endif
