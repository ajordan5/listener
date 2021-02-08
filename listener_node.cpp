#include "listener.h"

// void Listener::posesStampedCallback(ConstPosesStampedPtr &posesStamped)
// {
//   std::cout << "in callback \n";
//   // auto boat = py::module::import("boat");
//   // boat.attr("Boat");
//   // boat.attr("print_it")();
//   // std::cout << posesStamped->DebugString();

//   // ::google::protobuf::int32 sec = posesStamped->time().sec();
//   // ::google::protobuf::int32 nsec = posesStamped->time().nsec();
//   // std::cout << "Read time: sec: " << sec << " nsec: " << nsec << std::endl;

//   // for (int i =0; i < posesStamped->pose_size(); ++i)
//   // {
//   //   const ::gazebo::msgs::Pose &pose = posesStamped->pose(i);
//   //   std::string name = pose.name();
//   //   if (name == std::string("box"))
//   //   {
//   //     const ::gazebo::msgs::Vector3d &position = pose.position();

//   //     double x = position.x();
//   //     double y = position.y();
//   //     double z = position.z();

//   //     std::cout << "Read position: x: " << x
//   //         << " y: " << y << " z: " << z << std::endl;
//   //   }
//   // }
// }

int main(int _argc, char **_argv)
{
  Listener listener(_argc,_argv);
}