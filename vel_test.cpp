#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

// Gazebo's API has changed between major releases. These changes are
// accounted for with #if..#endif blocks in this file.
#if GAZEBO_MAJOR_VERSION < 6
#include <gazebo/gazebo.hh>
#else
#include <gazebo/gazebo_client.hh>
#endif

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
	// Load gazebo as a client
#if GAZEBO_MAJOR_VERSION < 6
	gazebo::setupClient(_argc, _argv);
#else
	gazebo::client::setup(_argc, _argv);
#endif

	// Create our node for communication
	gazebo::transport::NodePtr node(new gazebo::transport::Node());
	node->Init();

	// Publish to the robi topics
	gazebo::transport::PublisherPtr pub_left = node->Advertise<gazebo::msgs::Vector3d>("~/robi_model/robi_cmd_left");
	gazebo::transport::PublisherPtr pub_right = node->Advertise<gazebo::msgs::Vector3d>("~/robi_model/robi_cmd_right");

	// Wait for a subscriber to connect to this publisher
	pub_left->WaitForConnection();
	pub_right->WaitForConnection();

	// Create a a vector3 message
	gazebo::msgs::Vector3d msg_left;
	gazebo::msgs::Vector3d msg_right;

	// Set the velocity in the x-component
#if GAZEBO_MAJOR_VERSION < 6
	gazebo::msgs::Set(&msg_left, gazebo::math::Vector3(std::atof(_argv[1]), 0, 0));
	gazebo::msgs::Set(&msg_right, gazebo::math::Vector3(std::atof(_argv[2]), 0, 0));
#else
	gazebo::msgs::Set(&msg_left, ignition::math::Vector3d(std::atof(_argv[1]), 0, 0));
	gazebo::msgs::Set(&msg_right, ignition::math::Vector3d(std::atof(_argv[2]), 0, 0));
#endif

	// Send the message
	pub_left->Publish(msg_left);
	pub_right->Publish(msg_right);

	// Make sure to shut everything down.
#if GAZEBO_MAJOR_VERSION < 6
	gazebo::shutdown();
#else
	gazebo::client::shutdown();
#endif
}
