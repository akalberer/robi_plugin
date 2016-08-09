#ifndef _ROBI_PLUGIN_HPP_
#define _ROBI_PLUGIN_HPP_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/common/PID.hh>
#include <gazebo/physics/JointController.hh>
#include <gazebo/physics/physics.hh>
#include <string>

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
			//if(!this->joint_left->GetScopedName().compare(left_wheel_name)){
				//std::cerr << "left found\n";
			//}
  			this->model->GetJointController()->SetVelocityTarget(this->joint_left->GetScopedName(), velocity_left);
			this->model->GetJointController()->SetVelocityTarget(this->joint_right->GetScopedName(), velocity_right);
    		}

		private: physics::ModelPtr model;
		private: physics::JointPtr joint_left;
		private: common::PID pid_left;
		private: physics::JointPtr joint_right;
		private: common::PID pid_right;
		public: static const std::string left_wheel_name;
  	};

  	// Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  	GZ_REGISTER_MODEL_PLUGIN(RobiPlugin)
}
#endif
