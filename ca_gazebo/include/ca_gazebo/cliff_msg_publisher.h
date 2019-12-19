    #ifndef GAZEBO_CLIFF_MSG_PUBLISHER_HH
    #define GAZEBO_CLIFF_MSG_PUBLISHER_HH
    
    #include <ros/ros.h>
    
    #include <gazebo/gazebo.hh>
    #include <gazebo/physics/physics.hh>
    #include <gazebo/common/common.hh>
    #include <gazebo/common/Plugin.hh>
    #include <gazebo/common/Events.hh>

    #include <mutex>

    #include "std_msgs/Bool.h"
    #include "ca_msgs/Cliff.h"

    namespace gazebo
    {
        /// \brief A Cliff Msg publisher
        class GazeboCliffMsgPublisher : public ModelPlugin
    {
        public:

        /// \brief Load the plugin
        /// \param take in SDF root element
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf); 

        /// \brief Callback to the cliff sensors
        /// \param take pointer to the message
        void SideLeftCliffCallback(const std_msgs::Bool::ConstPtr& data); 
        void SideRightCliffCallback(const std_msgs::Bool::ConstPtr& data);
        void FrontLeftCliffCallback(const std_msgs::Bool::ConstPtr& data);
        void FrontRightCliffCallback(const std_msgs::Bool::ConstPtr& data);

        /// \brief Update the controller
        void OnUpdate();

        private:

        /// Pointer to the model
        physics::ModelPtr model;

        /// Pointer to the update event connection
        private: event::ConnectionPtr updateConnection;

        /// Initialize ROS variables
        ros::NodeHandle nh_;
        ros::Publisher publisher_;
        ros::Subscriber side_left_,side_right_, front_left_, front_right_;
        ros::Time prev_update_time_;
        event::ConnectionPtr updateConnection_;
        
        /// Auxiliar variables
        char robot_number_;
        int seq_counter_;
        std::string frame_name_;
        bool get_side_left_, get_side_right_, get_front_left_, get_front_right_;
        double update_period_;
        std::mutex side_left_mutex, side_right_mutex, front_left_mutex, front_right_mutex;
      }; // GazeboCliffMsgPublisher
    } // gazebo
    #endif // GAZEBO_CLIFF_MSG_PUBLISHER_HH
    