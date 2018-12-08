#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"

namespace gazebo
{
  class SaganPlugin : public ModelPlugin
  {
   /// \brief Constructor
    public: SaganPlugin() {}

    /// \brief The load function is called by Gazebo when the plugin is
    /// inserted into simulation
    /// \param[in] _model A pointer to the model that this plugin is
    /// attached to.
    /// \param[in] _sdf A pointer to the plugin's SDF element.	
	
     public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
	// Safety check
	  if (_model->GetJointCount() == 0)
	  {
	    std::cerr << "Invalid joint count, Sagan plugin not loaded\n";
	    return;
	  }
	  // Store the pointer to the model
	  this->model = _model;
	
	  // Setup a P-controller, with a gain of 0.1.
	  this->pid = common::PID(0.1, 0, 0);
	
	 this->joint1 = _model->GetJoints()[1];
 

	// Apply the P-controller to the joint1.
	  this->model->GetJointController()->SetVelocityPID(
	      this->joint1->GetScopedName(), this->pid);


	// Default to zero velocity
	double velocity = 0;

	// Check that the velocity element exists, then read the value
	if (_sdf->HasElement("velocity"))
	  velocity = _sdf->Get<double>("velocity");

	// Create the node
	this->node = transport::NodePtr(new transport::Node());
	#if GAZEBO_MAJOR_VERSION < 8
	this->node->Init(this->model->GetWorld()->GetName());
	#else
	this->node->Init(this->model->GetWorld()->Name());
	#endif

	// Create a topic name
	std::string topicName = "~/" + this->model->GetName() + "/vel_cmd";

	// Subscribe to the topic, and register a callback
	this->sub = this->node->Subscribe(topicName,
	   &SaganPlugin::OnMsg, this);
	// Initialize ros, if it has not already bee initialized.
	if (!ros::isInitialized())
	{
	  int argc = 0;
	  char **argv = NULL;
	  ros::init(argc, argv, "gazebo_client",
	      ros::init_options::NoSigintHandler);
	}

	// Create our ROS node. This acts in a similar manner to
	// the Gazebo node
	this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

	// Create a named topic, and subscribe to it.
	ros::SubscribeOptions so =
	  ros::SubscribeOptions::create<std_msgs::Float32>(
	      "/" + this->model->GetName() + "/vel_cmd",
	      1,
	      boost::bind(&SaganPlugin::OnRosMsg, this, _1),
	      ros::VoidPtr(), &this->rosQueue);
	this->rosSub = this->rosNode->subscribe(so);

	// Spin up the queue helper thread.
	this->rosQueueThread =
	  std::thread(std::bind(&SaganPlugin::QueueThread, this));
	
	std::cerr << "\nThe Sagan plugin is attach to model[" <<
        _model->GetName() << "]\n";


    }
	/// \brief Handle incoming message
	/// \param[in] _msg Repurpose a vector3 message. This function will
	/// only use the x component.
	private: void OnMsg(ConstVector3dPtr &_msg)
	{
	  this->SetVelocity(_msg->x());
	}
	

	/// \brief Set the velocity of the Sagan
	/// \param[in] _vel New target velocity
	public: void SetVelocity(const double &_vel)
	{
	  // Set the joint's target velocity.
	  this->model->GetJointController()->SetVelocityTarget(
	      this->joint1->GetScopedName(), _vel);
	}
	/// \brief Handle an incoming message from ROS
	/// \param[in] _msg A float value that is used to set the velocity
	/// of the Velodyne.
	public: void OnRosMsg(const std_msgs::Float32ConstPtr &_msg)
	{
	  this->SetVelocity(_msg->data);
	}

	/// \brief ROS helper function that processes messages
	private: void QueueThread()
	{
	  static const double timeout = 0.01;
	  while (this->rosNode->ok())
	  {
	    this->rosQueue.callAvailable(ros::WallDuration(timeout));
	  }
	}


   	/// \brief Pointer to the model.
	private: physics::ModelPtr model;

	/// \brief Pointer to the joint1.
	private: physics::JointPtr joint1;
	

	/// \brief A PID controller for the joint.
	private: common::PID pid;
	/// \brief A node used for transport
	private: transport::NodePtr node;

	/// \brief A subscriber to a named topic.
	private: transport::SubscriberPtr sub;
	
	/// \brief A node use for ROS transport
	private: std::unique_ptr<ros::NodeHandle> rosNode;

	/// \brief A ROS subscriber
	private: ros::Subscriber rosSub;

	/// \brief A ROS callbackqueue that helps process messages
	private: ros::CallbackQueue rosQueue;

	/// \brief A thread the keeps running the rosQueue
	private: std::thread rosQueueThread;
	
	  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(SaganPlugin)
}
