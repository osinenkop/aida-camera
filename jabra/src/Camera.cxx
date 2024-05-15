#include "Camera.hxx"
#include <sstream>

Camera::~Camera(){
    this -> core_.release();
}

void Camera::setIndex(std::string value){
    this -> data_.index_ = value;
}

void Camera::setSerial(std::string value){
    // if (value.size() > 0){value.pop_back();}
    this -> data_.serial_ = value;
}

void Camera::setPortNumber(std::string value){
    this -> data_.port_number_ = value;
}

void Camera::setResolution(std::string value){
    std::vector<std::string> res{this->splitString(value, ',')};
    this -> data_.image_size_.height = std::stoul(res[1]);
    this -> data_.image_size_.width = std::stoul(res[0]);
}

void Camera::setFPS(unsigned value){
    this -> data_.fps_ = value;
}

void Camera::setCalibrationStatus(bool value){
    this -> data_.is_calibrated_ = value;
}

void Camera::setInitialPose(std::string value){
    std::vector<std::string> pose_vec{this->splitString(value, ',')};
    for(int i{}; i < 3; i++){
        this -> data_.initial_pose_.at(i) = std::stod(pose_vec.at(i));
    } 
}

void Camera::setInitialOrientation(std::string value){
    std::vector<std::string> orientation_vec{this->splitString(value, ',')};
    for(int i{}; i < 3; i++){
        this -> data_.initial_orientation_.at(i) = std::stod(orientation_vec.at(i));
    } 
}

void Camera::setCameraMatrix(std::vector<double> value){
    data_.k = value;
    data_.camera_matrix_ = cv::Mat(value, true).reshape(1,3);
    data_.camera_matrix_.convertTo(data_.camera_matrix_, CV_32FC1);
}

auto Camera::setDistortionModel(std::string value) -> void{
    data_.distortion_model_ = value;
}

void Camera::setDistortionCoefficients(std::vector<double> value){
    data_.d = value;
    data_.distortion_coefficients_ = cv::Mat(value, true).reshape(1, 1);
    data_.distortion_coefficients_.convertTo(data_.distortion_coefficients_, CV_32FC1);
}

void Camera::setRectificationMatrix(std::vector<double> value){
    data_.r = value;
    data_.rectification_matrix_ = cv::Mat(value, true).reshape(1,3);
    data_.rectification_matrix_.convertTo(data_.rectification_matrix_, CV_32FC1);
}

void Camera::setProjectionMatrix(std::vector<double> value){
    data_.p = value;
    data_.projection_matrix_ = cv::Mat(value, true).reshape(1,3);
    data_.projection_matrix_.convertTo(data_.projection_matrix_, CV_32FC1);
}

double Camera::getFPS(){
    return this -> data_.fps_;
}

std::string Camera::getIndex(){
    return  this -> data_.index_;
}

std::vector<std::string> Camera::splitString(std::string& input, char delimiter){
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(input);
    while(std::getline(tokenStream, token, delimiter)){
        tokens.push_back(token);
    }
    return tokens;
}


void Camera::warmUp(){
    this -> data_.distorted_frame_ = cv::Mat(this -> data_.image_size_.height, this -> data_.image_size_.width, CV_8UC3);
    this -> data_.undistorted_frame_ = cv::Mat(this -> data_.image_size_.height, this -> data_.image_size_.width, CV_8UC3);

    cv::initUndistortRectifyMap(this -> data_.camera_matrix_, this -> data_.distortion_coefficients_, cv::Matx33f::eye(), this -> data_.camera_matrix_, this -> data_.image_size_, CV_32FC1, this -> data_.map1_, this -> data_.map2_);

    this -> core_ = cv::VideoCapture( this -> data_.port_number_, cv::CAP_V4L2);

    if (!this -> core_.isOpened()){
        std::cerr << "Could not open the camera.";
        throw std::runtime_error("Could not open the camera.");
    }

    this -> core_.set(cv::CAP_PROP_FRAME_WIDTH, this -> data_.image_size_.width);
    this -> core_.set(cv::CAP_PROP_FRAME_HEIGHT, this -> data_.image_size_.height);
    this -> data_.port_opened_ = this -> core_.open( this -> data_.port_number_);

    std::clog << "Model: " << "Jabra_PanaCast_20\n";
    std::clog << "Serial: " << this -> data_.serial_ << "\n";
    std::clog << "Port: " << this -> data_.port_number_ << "\n";
    std::clog << "Resolution: " << this -> data_.image_size_ << "\n";
    std::clog << "Camera Matrix: " << this -> data_.camera_matrix_ << std::endl;
}


void Camera::prepareFrame(){
     if (this -> core_.isOpened()){
        this -> core_ >> this -> data_.distorted_frame_;
        if (this -> data_.distorted_frame_.empty()){
            std::cerr << "Blank frame grabbed!";
            throw std::runtime_error("Blank frame grabbed!");
        }
        if(this -> data_.is_calibrated_){this -> undistortImage();}
     }
}

void Camera::undistortImage(){
    this -> data_.undistorted_frame_ = cv::Mat::zeros(this -> data_.image_size_.height, this -> data_.image_size_.width, CV_8UC3);
    cv::remap(this -> data_.distorted_frame_, this -> data_.undistorted_frame_, this -> data_.map1_, this -> data_.map2_, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());
}
