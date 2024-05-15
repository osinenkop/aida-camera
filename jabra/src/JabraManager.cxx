#include "JabraManager.hxx"

Jabra::Jabra(std::string node_name): Node(node_name){
    this -> declare_parameter("serial", "");
    this -> declare_parameter("port_number", "");
    this -> declare_parameter("index", "");
    this -> declare_parameter("resolution", "");
    this -> declare_parameter("fps", 0);
    this -> declare_parameter("is_calibrated", false);
    this -> declare_parameter("camera_matrix", std::vector<double>(9));
    this -> declare_parameter("distortion_model", "");
    this -> declare_parameter("distortion_coefficients", std::vector<double>(5));
    this -> declare_parameter("rectification_matrix", std::vector<double>(9));
    this -> declare_parameter("projection_matrix", std::vector<double>(12));
    this -> declare_parameter("position", "0.0,0.0,0.0");
    this -> declare_parameter("orientation", "0.0,0.0,0.0");
    this -> declare_parameter("queue_size", 10);

    this -> cam_.setSerial(this->get_parameter("serial").as_string());
    this -> cam_.setPortNumber(this->get_parameter("port_number").as_string());
    this -> cam_.setIndex(this->get_parameter("index").as_string());
    this -> cam_.setResolution(this->get_parameter("resolution").as_string());
    this -> cam_.setFPS(this->get_parameter("fps").as_int());
    this -> cam_.setCalibrationStatus(this->get_parameter("is_calibrated").as_bool());
    this -> cam_.setCameraMatrix(this->get_parameter("camera_matrix").as_double_array());
    this -> cam_.setDistortionModel(this->get_parameter("distortion_model").as_string());
    this -> cam_.setDistortionCoefficients(this->get_parameter("distortion_coefficients").as_double_array());
    this -> cam_.setRectificationMatrix(this->get_parameter("rectification_matrix").as_double_array());
    this -> cam_.setProjectionMatrix(this->get_parameter("projection_matrix").as_double_array());
    this -> cam_.setInitialPose(this->get_parameter("position").as_string());
    this -> cam_.setInitialOrientation(this->get_parameter("orientation").as_string());

    this -> queue_size_ = this->get_parameter("queue_size").as_int();
    // this -> frame_id_ = node_name;
    this -> frame_id_ = "Camera_" + this -> cam_.data_.index_;
}

Jabra::~Jabra(){
    std::cout << "Jabra Destructor!\n";
    this -> image_transport_publisher_.shutdown();
    this->sleep(500);
}

void Jabra::sleep(unsigned millisec){
    std::this_thread::sleep_for(std::chrono::milliseconds(millisec));
}

void Jabra::setPublisher(){
    this -> image_transport_ = std::make_shared<image_transport::ImageTransport>(this -> node_);
    this -> image_transport_publisher_ = this -> image_transport_->advertiseCamera("Camera_" + this -> cam_.data_.index_ + "/color" , this -> queue_size_);
}

void Jabra::warmUp(){
    // std::cout << this -> get_effective_namespace() <<"\n";
    node_ = this -> create_sub_node("jabra");

    this -> setPublisher();
    this -> cam_.warmUp();
}


void Jabra::getFrame(){
    this -> cam_.prepareFrame();
    this -> header_ = this -> getHeader();
    this -> getCameraInfo();

    if(this -> cam_.data_.is_calibrated_){
        this -> frame_ = cv_bridge::CvImage(this -> header_, "bgr8", this -> cam_.data_.undistorted_frame_).toImageMsg(); 
    }
    else{
        this -> frame_ = cv_bridge::CvImage(this -> header_, "bgr8", this -> cam_.data_.distorted_frame_).toImageMsg();
    }
}

auto Jabra::getCameraInfo() -> void{
    this -> camera_info_.header = this -> header_;
    this -> camera_info_.height = this -> cam_.data_.image_size_.height;
    this -> camera_info_.width  = this -> cam_.data_.image_size_.width;

    this -> camera_info_.distortion_model = this -> cam_.data_.distortion_model_;
    this -> camera_info_.d.resize(5);

    for(short i{}; i < 5; i++)
        this -> camera_info_.d[i] = cam_.data_.d[i];

    for(short i{}; i < 9; i++)
        this -> camera_info_.k[i] = cam_.data_.k[i];

    for(short i{}; i < 9; i++)
        this -> camera_info_.r[i] = cam_.data_.r[i];

    for(short i{}; i < 12; i++)
        this -> camera_info_.p[i] = cam_.data_.p[i];
}

auto Jabra::getHeader() -> std_msgs::msg::Header{
    std_msgs::msg::Header header{};
    rclcpp::Time now = rclcpp::Clock().now();
    header.stamp.sec = now.seconds();
    header.stamp.nanosec = now.nanoseconds();
    header.frame_id = this -> frame_id_;

    return header;
}

void Jabra::loopForEver(){
    std::chrono::milliseconds fps_duration{static_cast<unsigned>(1000.0/this -> cam_.getFPS())};
    
    if(rclcpp::ok()){
        timer_ = this->create_wall_timer(fps_duration, std::bind(&Jabra::operate, this));
    }
}

void Jabra::operate(){
    this -> getFrame();
    this -> publishFrame();
}


void Jabra::publishFrame(){
    this -> image_transport_publisher_.publish(*this -> frame_, camera_info_);
}

void Jabra::log(std::string phrase){
    std::string final_string{"[INFO]" + '[' + this -> frame_id_ + "] " + phrase};
    RCLCPP_INFO(this->get_logger(), final_string.c_str());
}

void Jabra::log_err(std::string phrase){
    std::string final_string{"[ERROR] " + phrase};
    RCLCPP_ERROR(this->get_logger(), final_string.c_str());
}


    
