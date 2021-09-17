/*
 * Copyright (C) 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include "listener.h"

Listener::Listener(int _argc, char **_argv)
{
  py::scoped_interpreter python;

  py::object Boat = py::module::import("boat").attr("Boat");
  boat_ = Boat();
  py::object Rover = py::module::import("rover").attr("Rover");
  rover_ = Rover();

  // Load gazebo
  gazebo::client::setup(_argc, _argv);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Listen to Gazebo pose info topic
  //posesSubscriber = node->Subscribe("~/pose/info", &Listener::posesStampedCallback, this);
  gazebo::transport::SubscriberPtr sub = node->Subscribe("~/pose/info", &Listener::posesStampedCallback, this);
  
  // // Busy wait loop...replace with your own code as needed.
  while (true)
    gazebo::common::Time::MSleep(10);

  // Make sure to shut everything down.
  // The whole program won't shutdown.  Use cntrl z to stop the program and kill -9 %<#> to kill it.
  gazebo::client::shutdown();
}

Listener::~Listener()
{
}

void Listener::posesStampedCallback(ConstPosesStampedPtr &posesStamped)
{
  for (int i =0; i < posesStamped->pose_size(); ++i)
  {
    const ::gazebo::msgs::Pose &pose = posesStamped->pose(i);
    std::string name = pose.name();
    if (name == std::string("box"))
    {
      const ::gazebo::msgs::Vector3d &position = pose.position();
      const ::gazebo::msgs::Quaternion &orientation = pose.orientation();
      boat_.attr("update")(position.x(),position.y(),position.z(),orientation.x(),orientation.y(),orientation.z(),orientation.w());
    }
    if (name == std::string("iris"))
    {
      const ::gazebo::msgs::Vector3d &position = pose.position();
      const ::gazebo::msgs::Quaternion &orientation = pose.orientation();
      rover_.attr("update")(position.x(),position.y(),position.z(),orientation.x(),orientation.y(),orientation.z(),orientation.w());
    }
  }
}


