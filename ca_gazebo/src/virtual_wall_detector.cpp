#include "ca_gazebo/virtual_wall_detector.h"

#include <gazebo/common/Plugin.hh>

static const char create2_model_name_prefix[] = "irobot_create2.";
static const size_t create2_model_name_prefix_length = sizeof(create2_model_name_prefix) - 1;

namespace gazebo
{
  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(GazeboVirtualWallDetector)

  void GazeboVirtualWallDetector::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
  {
    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }
    if (!ros::isInitialized()) 
    {
      int argc = 0;
      char** argv = NULL;
      ros::init(argc, argv, "virtual_wall_detector");
    }
    // Initialize the ros variables and gaebo variables
    const std::string & name = _parent->GetName();
    std::istringstream iss(name.substr(create2_model_name_prefix_length));
    size_t x;
    iss >> x;
//    ROS_WARN("The name %s",name.c_str());
    if (iss && iss.eof()) 
    {
      std::ostringstream oss;
      oss << "create" << x << "/virtual_wall/raw/";
      std::string topic_name_ = oss.str();
      ROS_WARN("The name %s",oss.str().c_str());
      this->subscriber_ = this->nh_.subscribe(topic_name_, 1, &GazeboVirtualWallDetector::WallCallback, this);
      oss.str("");
      oss.clear();
      oss << "create" << x << "/virtual_wall/";
      topic_name_ = oss.str();
      this->publisher_ = this->nh_.advertise<std_msgs::Bool>(topic_name_, 1);
      this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&GazeboVirtualWallDetector::OnUpdate, this));    
      this->prev_update_time_ = ros::Time::now();
      this->update_period_ = 1.0/(_sdf->Get<double>("updateRate"));
    }
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Update the controller
  void GazeboVirtualWallDetector::OnUpdate()
  //Every update check if one of the sensors is active, if it is, change the output to true, publishes every 1ms
  {
    this->is_vwall_detected_ += this->get_vwall_;
    if ((ros::Time::now() - this->prev_update_time_) < ros::Duration(this->update_period_)) 
    {
      return;
    }
    this->publisher_.publish(this->is_vwall_detected_);
    this->is_vwall_detected_ = false;
    this->prev_update_time_ = ros::Time::now();
  }

  void GazeboVirtualWallDetector::WallCallback(const std_msgs::Bool::ConstPtr& data)
  // Callback to get the data from the virtual wall sensors
  {
    mtx.lock();
    this->get_vwall_ = data->data;
    mtx.unlock();
  }
}
