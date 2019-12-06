#include "ca_gazebo/virtual_wall_detector.h"

#include <gazebo/common/Plugin.hh>


namespace gazebo
{
  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(GazeboVirtualWallDetector)

  ////////////////////////////////////////////////////////////////////////////////
  // Constructor
  GazeboVirtualWallDetector::GazeboVirtualWallDetector():
    nh_("virtual_wall_detector_plugin")
{

}
  ////////////////////////////////////////////////////////////////////////////////
  // Destructor
  GazeboVirtualWallDetector::~GazeboVirtualWallDetector()
  {
  }

  void GazeboVirtualWallDetector::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
  {
    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
        << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }
    // Initialize the ros variables and gaebo variables
    this->virtual_wall_publisher_ = nh_.advertise<std_msgs::Bool>("/filtered/virtual_wall", 1);
    this->wall_subscriber_ = nh_.subscribe("/create1/virtual_wall", 1, &GazeboVirtualWallDetector::WallCallback, this);
    this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&GazeboVirtualWallDetector::OnUpdate, this));    
    this->prev_update_time_ = ros::Time::now();
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Update the controller
  void GazeboVirtualWallDetector::OnUpdate()
  //Every update check if one of the sensors is active, if it is, change the output to true, publishes every 1ms
  {
    this->virtual_wall_status += this->virtual_wall_sub;
    if ((ros::Time::now() - this->prev_update_time_) < ros::Duration(0.1)) 
    {
      return;
    }
    this->virtual_wall_publisher_.publish(this->virtual_wall_status);
    this->virtual_wall_status = false;
    this->prev_update_time_ = ros::Time::now();
  }

  void GazeboVirtualWallDetector::WallCallback(const std_msgs::Bool::ConstPtr& data)
  // Callback to get the data from the virtual wall sensors
  {
    this->virtual_wall_sub = data->data;
  }
}