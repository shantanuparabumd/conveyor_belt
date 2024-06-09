
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <conveyor_belt/conveyor_belt_plugin.hpp>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>


#include <memory>

namespace gazebo{


class ConveyorBeltPluginPrivate
{
  /**
   * @brief Class to hold data members and methods for plugin
   * 
   */
public:
  /// Connection to world update event. Callback is called while this is alive.
  gazebo::event::ConnectionPtr update_connection_;

  /// Node for ROS communication.
  gazebo_ros::Node::SharedPtr ros_node_;

  /// The joint that controls the movement of the belt.
  gazebo::physics::JointPtr belt_joint_;

  // Velocity to move the link
  double belt_velocity_;
  double max_velocity_;

  /// Position limit of belt joint to reset 
  double limit_;

  // Callback function to perform task with each iteration in Gazebo
  void OnUpdate();

  void OnPower(const std_msgs::msg::Bool::SharedPtr msg);


  // Subscriber to get the state of conveyor belt
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr power_subscriber;


};


/**
 * @brief Construct a new Conveyor Belt Plugin:: Conveyor Belt Plugin object
 * 
 */
ConveyorBeltPlugin::ConveyorBeltPlugin()
: impl_(std::make_unique<ConveyorBeltPluginPrivate>())
{
  printf("Initiated Conveyor Belt Plugin !\n");
}

/**
 * @brief Destroy the Conveyor Belt Plugin:: Conveyor Belt Plugin object
 * 
 */
ConveyorBeltPlugin::~ConveyorBeltPlugin()
{
}

/**
 * @brief Load the SDF/URDF model of the robot and access the links/joints.
 * 
 * @param model 
 * @param sdf 
 */
void ConveyorBeltPlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  // Create ROS node
  impl_->ros_node_ = gazebo_ros::Node::Get(sdf);


  impl_->power_subscriber = impl_->ros_node_->create_subscription<std_msgs::msg::Bool>("conveyor_power", 
    10, std::bind(&ConveyorBeltPluginPrivate::OnPower, impl_.get(), std::placeholders::_1));


  // Create/Get the  belt joint from the robot URDF
  impl_->belt_joint_ = model->GetJoint("belt_joint");

  // Exception handler to handle unavailable link 
  if (!impl_->belt_joint_) {
    RCLCPP_ERROR(impl_->ros_node_->get_logger(), "Belt joint not found, unable to start conveyor plugin");
    return;
  }

  // Set velocity (m/s)
  impl_->max_velocity_ = sdf->GetElement("max_velocity")->Get<double>();

  // Set limit (m)
  // Accessed from the URDF from joint limit
  impl_->limit_ = impl_->belt_joint_->UpperLimit();

  // Create a connection so the OnUpdate function is called at every simulation iteration. 
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&ConveyorBeltPluginPrivate::OnUpdate, impl_.get()));
}


/**
 * @brief This method is called at every time interval in Gazebo
 * 
 */
void ConveyorBeltPluginPrivate::OnUpdate()
{

  // Set the velocity of the belt
  belt_joint_->SetVelocity(0, belt_velocity_);


  // Get the current position of the belt
  double belt_position = belt_joint_->Position(0);

  // Set the Position of the belt to 0 if it exceeds the Upper Limit
  if (belt_position >= limit_){
    belt_joint_->SetPosition(0, 0);
  }

}


/**
 * @brief Callback function to get the state of the conveyor belt
 * 
 * @param msg 
 */
void ConveyorBeltPluginPrivate::OnPower(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data == false){
    belt_velocity_ = 0;
  }
  else{
    belt_velocity_ = max_velocity_;
  }
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(ConveyorBeltPlugin)

}