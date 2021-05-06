#include "plugin_depth.h"

#include <cmath>
#include <iostream>
#include <stdlib.h>

namespace gazebo
{
// Contructor
DepthSensor::DepthSensor()
{
}

// Destructor
DepthSensor::~DepthSensor()
{
  node_handle_->shutdown();
  delete node_handle_;
}

// Load the plugin
void DepthSensor::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  world = _model->GetWorld();
  link = _model->GetLink();
  link_name_ = link->GetName();

  if (_sdf->HasElement("bodyName"))
  {
    link_name_ = _sdf->GetElement("bodyName")->Get<std::string>();
    link = _model->GetLink(link_name_);
  }

  if (!link)
  {
    ROS_FATAL("plugin_depth error: bodyName: %s does not exist\n", link_name_.c_str());
    return;
  }

  // default parameters
  frame_id_ = link->GetName();
  depth_topic_ = "/auv_v2/depth";
  noise_ = 0.01;

  // load parameters
  if (_sdf->HasElement("frameId"))
    frame_id_ = _sdf->GetElement("frameId")->Get<std::string>();
  if (_sdf->HasElement("depthTopicName"))
    depth_topic_ = _sdf->GetElement("depthTopicName")->Get<std::string>();
  if (_sdf->HasElement("noise"))
    noise_ = _sdf->GetElement("noise")->Get<double>();
  depth_.header.frame_id = frame_id_;

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable "
                     "to load plugin. "
                     << "Load the Gazebo system plugin "
                        "'libgazebo_ros_api_plugin.so' in the gazebo_ros "
                        "package)");
    return;
  }

  node_handle_ = new ros::NodeHandle;

  // advertise depth
  if (!depth_topic_.empty())
  {
    depth_publisher_ = node_handle_->advertise<geometry_msgs::PointStamped>(depth_topic_, 1024);
  }

  // connect Update function
  updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&DepthSensor::Update, this));

  ROS_INFO("Loaded DepthSensor plugin.");
}

void DepthSensor::Reset()
{
}

// Update
void DepthSensor::Update()
{
  // Get new command
  callback_queue_.callAvailable();

#if (GAZEBO_MAJOR_VERSION >= 8)
  common::Time sim_time = world->SimTime();
#else
  common::Time sim_time = world->GetSimTime();
#endif

#if (GAZEBO_MAJOR_VERSION >= 8)
  ignition::math::Pose3d pose = link->WorldPose();
  double depth = pose.Pos().Z();
#else
  math::Pose pose = link->GetWorldPose();
  double depth = pose.pos.z;
#endif

  // add noise to the actual value
  depth += noise_ * drand48();

  depth_.header.stamp = ros::Time::now();
  depth_.point.z = depth;
  depth_publisher_.publish(depth_);
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(DepthSensor)

}  // namespace gazebo
