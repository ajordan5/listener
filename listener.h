#ifndef LISTENER_H
#define LISTENER_H

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

#include <iostream>
#include <pybind11/pybind11.h>
#include <pybind11/embed.h>

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

namespace py = pybind11;

class Listener
{
public:
    Listener(int _argc, char **_argv);
    ~Listener();
    gazebo::transport::SubscriberPtr posesSubscriber;
    void posesStampedCallback(ConstPosesStampedPtr &posesStamped);
    py::object boat_;
    // Listener::posesStampedCallback(ConstPosesStampedPtr &posesStamped);
};

#endif