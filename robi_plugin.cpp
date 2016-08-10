#ifndef _ROBI_PLUGIN_HPP_
#define _ROBI_PLUGIN_HPP_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/common/PID.hh>
#include <gazebo/physics/JointController.hh>
#include <gazebo/physics/physics.hh>
#include <string>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <thread>
#include <gazebo/math/Pose.hh>

namespace gazebo
{
	/// \brief Plugin to control the Robi
	class RobiPlugin : public ModelPlugin
	{
    		/// \brief Constructor
    		public: RobiPlugin() {
			const std::string left_wheel_name = "robi_model::robi::left_wheel_hinge";
		}
	
	    	/// \brief The load function is called by Gazebo when the plugin is
	    	/// inserted into simulation
	    	/// \param[in] _model A pointer to the model that this plugin is
	    	/// attached to.	
	    	/// \param[in] _sdf A pointer to the plugin's SDF element.
  	  	public: virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
		{
 			// Just output a message for now
      			std::cerr << "\nThe velodyne plugin is attach to model[" <<
        		_model->GetName() << "]\n";
			// Safety check
			if (_model->GetJointCount() == 0)
  			{
			    	std::cerr << "Invalid joint count, Velodyne plugin not loaded\n";
				return;
			}
			std::cerr << "nof joints: " << _model->GetJointCount();

  			// Store the model pointer for convenience.
			this->model = _model;

  			// Get the first joint. We are making an assumption about the model
			// that left is always first

			
  			this->joint_left = _model->GetJoint("robi_model::robi::left_wheel_hinge");
			this->joint_right = _model->GetJoint("robi_model::robi::right_wheel_hinge");

  			// Setup a P-controller, with a gain of 0.1.
  			this->pid_left = common::PID(0.1, 0, 0);
  			this->pid_right = common::PID(0.1, 0, 0);

			double velocity_left = 0;
			double velocity_right = 0;
			if(_sdf->HasElement("velocity_left")){
				velocity_left = _sdf->Get<double>("velocity_left");
			}
			if(_sdf->HasElement("velocity_right")){
				velocity_right = _sdf->Get<double>("velocity_right");
			}

  			// Apply the P-controller to the joint.
  			this->model->GetJointController()->SetVelocityPID(this->joint_left->GetScopedName(), this->pid_left);
			this->model->GetJointController()->SetVelocityPID(this->joint_right->GetScopedName(), this->pid_right);

  			// Set the joint's target velocity. This target velocity is just
  			// for demonstration purposes.
			std::cerr << "name joint: " << this->joint_left->GetScopedName() << std::endl;
			
  			this->model->GetJointController()->SetVelocityTarget(this->joint_left->GetScopedName(), velocity_left);
			this->model->GetJointController()->SetVelocityTarget(this->joint_right->GetScopedName(), velocity_right);

			// Create the node
			this->node = transport::NodePtr(new transport::Node());
			this->node->Init(this->model->GetWorld()->GetName());

			std::cerr << "model name: " << this->model->GetName() << "\n";
			// Create topic names
			std::string topicNameLeft = "~/" + this->model->GetName() + "/robi_cmd_left";
			std::string topicNameRight = "~/" + this->model->GetName() + "/robi_cmd_right";

			// Subscribe to the topic, and register a callback
			this->sub_left = this->node->Subscribe(topicNameLeft, &RobiPlugin::OnMsgLeft, this);
			this->sub_right = this->node->Subscribe(topicNameRight, &RobiPlugin::OnMsgRight, this);

			// Initialize ros, if it has not already bee initialized.
			if (!ros::isInitialized())
			{
  				int argc = 0;
  				char **argv = NULL;
  				ros::init(argc, argv, "gazebo_client",
      				ros::init_options::NoSigintHandler);
			}

			// Create our ROS node. This acts in a similar manner to the Gazebo node
			this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

			// Create a named topic, and subscribe to it.
			ros::SubscribeOptions so_left = ros::SubscribeOptions::create<std_msgs::Float32>("/" + this->model->GetName() + "/robi_cmd_left", 1, boost::bind(&RobiPlugin::OnRosMsgLeft, this, _1), ros::VoidPtr(), &this->rosQueue_left);
			this->rosSub_left = this->rosNode->subscribe(so_left);
			ros::SubscribeOptions so_right = ros::SubscribeOptions::create<std_msgs::Float32>("/" + this->model->GetName() + "/robi_cmd_right", 1, boost::bind(&RobiPlugin::OnRosMsgRight, this, _1), ros::VoidPtr(), &this->rosQueue_right);
			this->rosSub_right = this->rosNode->subscribe(so_right);

			this->rosPub_pose_x = this->rosNode->advertise<std_msgs::Float64>("/" + this->model->GetName() + "/robi_pose/x", 1);
			this->rosPub_pose_y = this->rosNode->advertise<std_msgs::Float64>("/" + this->model->GetName() + "/robi_pose/y", 1);
			// Spin up the queue helper thread.
			this->rosQueueThread = std::thread(std::bind(&RobiPlugin::QueueThread, this));
    		}

		/// \brief Set the velocity of the Robi
		/// \param[in] vel_left New target velocity for left wheel
		/// \param[in] vel_right New target velocity for right wheel
		public: void SetVelocity(const double &vel_left, const double vel_right)
		{
			// Set the joint's target velocity.
			this->model->GetJointController()->SetVelocityTarget(this->joint_left->GetScopedName(), vel_left);
			this->model->GetJointController()->SetVelocityTarget(this->joint_right->GetScopedName(), vel_right);
		}

		/// \brief Set the velocity of the Robi
		/// \param[in] vel_left New target velocity for left wheel
		public: void SetVelocityLeft(const double &vel_left)
		{
			// Set the joint's target velocity.
			this->model->GetJointController()->SetVelocityTarget(this->joint_left->GetScopedName(), vel_left);
		}

		/// \brief Set the velocity of the Robi
		/// \param[in] vel_right New target velocity for right wheel
		public: void SetVelocityRight(const double &vel_right)
		{
			// Set the joint's target velocity.
			this->model->GetJointController()->SetVelocityTarget(this->joint_right->GetScopedName(), vel_right);
		}

		/// \brief Handle incoming message
		/// \param[in] msg_left Repurpose a vector3 message. This function will
		/// only use the x component.
		private: void OnMsgLeft(ConstVector3dPtr &msg_left)
		{
			this->SetVelocityLeft(msg_left->x());
		}

		/// \brief Handle incoming message
		/// \param[in] msg_right Repurpose a vector3 message. This function will
		/// only use the x component.
		private: void OnMsgRight(ConstVector3dPtr &msg_right)
		{
			this->SetVelocityRight(msg_right->x());
		}

		/// \brief Handle an incoming message from ROS
		/// \param[in] msg A float value that is used to set the velocity of the left Robi wheel
		public: void OnRosMsgLeft(const std_msgs::Float32ConstPtr &msg_left)
		{
			this->SetVelocityLeft(msg_left->data);
		}

		/// \brief Handle an incoming message from ROS
		/// \param[in] msg A float value that is used to set the velocity of the left Robi wheel
		public: void OnRosMsgRight(const std_msgs::Float32ConstPtr &msg_right)
		{
			this->SetVelocityRight(msg_right->data);
		}

		/// \brief ROS helper function that processes messages
		private: void QueueThread()
		{
			static const double timeout = 0.01;
			while (this->rosNode->ok())
			{
				this->rosQueue_left.callAvailable(ros::WallDuration(timeout));
				this->rosQueue_right.callAvailable(ros::WallDuration(timeout));
				gazebo::math::Pose pose = this->model->GetWorldPose();
				std_msgs::Float64 msg_x;
				msg_x.data = pose.pos.x;
				std_msgs::Float64 msg_y;
				msg_y.data = pose.pos.y;
				rosPub_pose_x.publish(msg_x);
				rosPub_pose_y.publish(msg_y);
			}
		}

		private: physics::ModelPtr model;
		private: physics::JointPtr joint_left;
		private: common::PID pid_left;
		private: physics::JointPtr joint_right;
		private: common::PID pid_right;
		public: static const std::string left_wheel_name;
		/// \brief A node used for transport
		private: transport::NodePtr node;
		/// \brief A subscriber to a named topic.
		private: transport::SubscriberPtr sub_left;
		private: transport::SubscriberPtr sub_right;
		
		// ROS variables		
		/// \brief A node use for ROS transport
		private: std::unique_ptr<ros::NodeHandle> rosNode;
		/// \brief ROS subscriber
		private: ros::Subscriber rosSub_left;
		private: ros::Subscriber rosSub_right;
		private: ros::Publisher rosPub_pose_x;
		private: ros::Publisher rosPub_pose_y;
		/// \brief ROS callbackqueue that helps process messages
		private: ros::CallbackQueue rosQueue_left;
		private: ros::CallbackQueue rosQueue_right;
		/// \brief A thread the keeps running the rosQueue
		private: std::thread rosQueueThread;

  	};

  	// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  	GZ_REGISTER_MODEL_PLUGIN(RobiPlugin)
}
#endif
