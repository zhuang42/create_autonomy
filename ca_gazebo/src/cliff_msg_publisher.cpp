#include "ca_gazebo/cliff_msg_publisher.h"

namespace gazebo
{
  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(GazeboCliffMsgPublisher)

  void GazeboCliffMsgPublisher::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
  {
    this->update_period_ = 1.0/(_sdf->Get<double>("updateRate"));
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
      ros::init(argc, argv, "cliff_msg_publisher");
    }
    // Initialize the ros variables and gazebo variables
    this->robot_number_ =_sdf->Get<char>("robotNumber");
    if (!_sdf->HasElement("frameName"))
    {
        ROS_INFO("cliff plugin missing <frameName>, defaults to world");
        this->frame_name_ = "base_footprint";
    }
    else
        this->frame_name_ = _sdf->Get<std::string>("frameName");

    std::ostringstream oss;
    oss << "create" << this->robot_number_ << "/side_left_cliff_sensor/raw/";
    std::string topic_name = oss.str();
    this->side_left_ = this->nh_.subscribe(topic_name, 1, &GazeboCliffMsgPublisher::SideLeftCliffCallback, this);
    oss.str("");
    oss << "create" << this->robot_number_ << "/side_right_cliff_sensor/raw/";
    topic_name = oss.str();
    this->side_right_ = this->nh_.subscribe(topic_name, 1, &GazeboCliffMsgPublisher::SideRightCliffCallback, this);
    oss.str("");
    oss << "create" << this->robot_number_ << "/front_left_cliff_sensor/raw/";
    topic_name = oss.str();
    this->front_left_ = this->nh_.subscribe(topic_name, 1, &GazeboCliffMsgPublisher::FrontLeftCliffCallback, this);
    oss.str("");
    oss << "create" << this->robot_number_ << "/front_right_cliff_sensor/raw/";
    topic_name = oss.str();
    this->front_right_ = this->nh_.subscribe(topic_name, 1, &GazeboCliffMsgPublisher::FrontRightCliffCallback, this);
    oss.str("");                  
    oss << "create" << this->robot_number_ << "/cliff_msg/";
    topic_name = oss.str();
    this->publisher_ = this->nh_.advertise<ca_msgs::Cliff>(topic_name, 1);
    this->seq_counter_ = 0;
    this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&GazeboCliffMsgPublisher::OnUpdate, this));    
    this->prev_update_time_ = ros::Time::now();
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Update the controller
  void GazeboCliffMsgPublisher::OnUpdate()
  //Every update check if one of the sensors is active, if it is, change the output to true, publishes every 1ms
  {
    if ((ros::Time::now() - this->prev_update_time_) < ros::Duration(this->update_period_)) 
    {
      return;
    }
    ca_msgs::Cliff msg;
    this->seq_counter_++;
    msg.header.frame_id = this->frame_name_;
    msg.header.seq = this->seq_counter_;
    msg.header.stamp = ros::Time::now();
    {
      const std::lock_guard<std::mutex> lock(side_left_mutex);
      msg.is_cliff_left = this->get_side_left_;
    }
      const std::lock_guard<std::mutex> lock(side_right_mutex);
      msg.is_cliff_right = this->get_side_right_;   
    {
      const std::lock_guard<std::mutex> lock(front_left_mutex);
      msg.is_cliff_front_left = this->get_front_left_;
    }

    {
      const std::lock_guard<std::mutex> lock(front_right_mutex);
      msg.is_cliff_front_right = this->get_front_right_;
    }
    this->publisher_.publish(msg);
    this->prev_update_time_ = ros::Time::now();
  }

  void GazeboCliffMsgPublisher::SideLeftCliffCallback(const std_msgs::Bool::ConstPtr& data)
  // Callback to get the data from the side left cliff sensor
  {
    const std::lock_guard<std::mutex> lock(side_left_mutex);
    this->get_side_left_ = data->data;
  }

    void GazeboCliffMsgPublisher::SideRightCliffCallback(const std_msgs::Bool::ConstPtr& data)
  // Callback to get the data from the side right cliff sensor
  {
    const std::lock_guard<std::mutex> lock(side_right_mutex);
    this->get_side_right_ = data->data;
  }

    void GazeboCliffMsgPublisher::FrontLeftCliffCallback(const std_msgs::Bool::ConstPtr& data)
  // Callback to get the data from the front left cliff sensor
  {
    const std::lock_guard<std::mutex> lock(front_left_mutex);
    this->get_front_left_ = data->data;
  }

    void GazeboCliffMsgPublisher::FrontRightCliffCallback(const std_msgs::Bool::ConstPtr& data)
  // Callback to get the data from the front right sensor
  {
    const std::lock_guard<std::mutex> lock(front_right_mutex);
    this->get_front_right_ = data->data;
  }
}
