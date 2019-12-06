    #ifndef GAZEBO_VIRTUAL_WALL_DETECTOR_HH
    #define GAZEBO_VIRTUAL_WALL_DETECTOR_HH
    
    #include <ros/ros.h>
    
    
    #include <gazebo/gazebo.hh>
    #include <gazebo/physics/physics.hh>
    #include <gazebo/common/common.hh>

    #include "std_msgs/Bool.h"

    namespace gazebo
    {
      /// \brief A Virtual Wall publisher
      class GazeboVirtualWallDetector : public ModelPlugin
    {
        public:

        ///Constructor
        GazeboVirtualWallDetector();   

        ///Destructor
        ~GazeboVirtualWallDetector();

    /// \brief Load the plugin
    /// \param take in SDF root element
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf); 

    /// \brief Callback to the virtual wall sensors
    /// \param take pointer to the message
        void WallCallback(const std_msgs::Bool::ConstPtr& data); 

    /// \brief Update the controller
        void OnUpdate();

        private:

    /// Pointer to the model
        physics::ModelPtr model;

    /// Pointer to the update event connection
        private: event::ConnectionPtr updateConnection;

    /// Initialize ROS variables
        ros::NodeHandle nh_;
        ros::Publisher virtual_wall_publisher_;
        ros::Subscriber wall_subscriber_;
        ros::Time prev_update_time_;
        event::ConnectionPtr updateConnection_;
        
    ///  Auxiliar variables
        bool virtual_wall_status;
        bool virtual_wall_sub;
      }; // GazeboVirtualWallDetector
    } // gazebo
    #endif // GAZEBO_VIRTUAL_WALL_DETECTOR_HH