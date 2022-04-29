#include "pepper.h"
#include "config.h"
#include "constraints/BoundedAccelerationConstr.h"
#include "devices/Speaker.h"
#include "devices/TouchSensor.h"
#include "devices/VisualDisplay.h"
#include "tasks/CoMRelativeBodyTask.h"

#include <RBDyn/parsers/urdf.h>

#include <mc_rtc/logging.h>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <fstream>

namespace bfs = boost::filesystem;

#ifndef M_PI
#  include <boost/math/constants/constants.hpp>
#  define M_PI boost::math::constants::pi<double>()
#endif

namespace mc_robots
{

PepperRobotModule::PepperRobotModule(bool fixed, bool hands, bool extraHardware)
: RobotModule(PEPPER_DESCRIPTION_PATH, "pepper")
{
  /* Path to surface descriptions */
  rsdf_dir = path + "/rsdf";

  /* Virtual links without convex files */
  virtualLinks.push_back("base_link");
  virtualLinks.push_back("tablet");
  virtualLinks.push_back("r_gripper");
  virtualLinks.push_back("RFinger11_link");
  virtualLinks.push_back("RFinger12_link");
  virtualLinks.push_back("RFinger13_link");
  virtualLinks.push_back("RFinger21_link");
  virtualLinks.push_back("RFinger22_link");
  // Using RFinger23_link for collision
  virtualLinks.push_back("RFinger31_link");
  virtualLinks.push_back("RFinger32_link");
  virtualLinks.push_back("RFinger33_link");
  virtualLinks.push_back("RFinger41_link");
  virtualLinks.push_back("RFinger42_link");
  virtualLinks.push_back("RFinger43_link");
  virtualLinks.push_back("RThumb1_link");
  virtualLinks.push_back("l_gripper");
  virtualLinks.push_back("LFinger11_link");
  virtualLinks.push_back("LFinger12_link");
  virtualLinks.push_back("LFinger13_link");
  virtualLinks.push_back("LFinger21_link");
  virtualLinks.push_back("LFinger22_link");
  // Using LFinger23_link for collision
  virtualLinks.push_back("LFinger31_link");
  virtualLinks.push_back("LFinger32_link");
  virtualLinks.push_back("LFinger33_link");
  virtualLinks.push_back("LFinger41_link");
  virtualLinks.push_back("LFinger42_link");
  virtualLinks.push_back("LFinger43_link");
  virtualLinks.push_back("LThumb1_link");
  virtualLinks.push_back("WheelFL_link");
  virtualLinks.push_back("WheelFR_link");
  virtualLinks.push_back("WheelB_link");
  virtualLinks.push_back("CameraTop_frame");
  virtualLinks.push_back("CameraTop_optical_frame");
  if(extraHardware)
  {
    // D435
    virtualLinks.push_back("RealSence_color_frame");
    virtualLinks.push_back("RealSence_color_optical_frame");
    virtualLinks.push_back("RealSence_depth_frame");
    virtualLinks.push_back("RealSence_depth_optical_frame");
    // T265
    virtualLinks.push_back("t265_pose");
    // Kinect
    virtualLinks.push_back("camera_visor");
    virtualLinks.push_back("kinect_depth_optical_frame");
    virtualLinks.push_back("kinect_imu_frame");
  }

  /* Gripper links not included in NoHands model */
  if(!hands)
  {
    excludedLinks.push_back("r_gripper");
    excludedLinks.push_back("RFinger11_link");
    excludedLinks.push_back("RFinger12_link");
    excludedLinks.push_back("RFinger13_link");
    excludedLinks.push_back("RFinger21_link");
    excludedLinks.push_back("RFinger22_link");
    excludedLinks.push_back("RFinger23_link");
    excludedLinks.push_back("RFinger31_link");
    excludedLinks.push_back("RFinger32_link");
    excludedLinks.push_back("RFinger33_link");
    excludedLinks.push_back("RFinger41_link");
    excludedLinks.push_back("RFinger42_link");
    excludedLinks.push_back("RFinger43_link");
    excludedLinks.push_back("RThumb1_link");
    excludedLinks.push_back("RThumb2_link");
    excludedLinks.push_back("l_gripper");
    excludedLinks.push_back("LFinger11_link");
    excludedLinks.push_back("LFinger12_link");
    excludedLinks.push_back("LFinger13_link");
    excludedLinks.push_back("LFinger21_link");
    excludedLinks.push_back("LFinger22_link");
    excludedLinks.push_back("LFinger23_link");
    excludedLinks.push_back("LFinger31_link");
    excludedLinks.push_back("LFinger32_link");
    excludedLinks.push_back("LFinger33_link");
    excludedLinks.push_back("LFinger41_link");
    excludedLinks.push_back("LFinger42_link");
    excludedLinks.push_back("LFinger43_link");
    excludedLinks.push_back("LThumb1_link");
    excludedLinks.push_back("LThumb2_link");
  }

  /* Extra hardware links only to include to ExtraHardware model  */
  if(!extraHardware)
  {
    excludedLinks.push_back("RealSense_screw_frame");
    excludedLinks.push_back("RealSence_color_frame");
    excludedLinks.push_back("RealSence_color_optical_frame");
    excludedLinks.push_back("RealSence_depth_frame");
    excludedLinks.push_back("RealSence_depth_optical_frame");
    excludedLinks.push_back("camera_body");
    excludedLinks.push_back("camera_base");
    excludedLinks.push_back("camera_visor");
    excludedLinks.push_back("kinect_depth_optical_frame");
    excludedLinks.push_back("kinect_imu_frame");
    excludedLinks.push_back("t265");
    excludedLinks.push_back("t265_pose");
  }

  /* Gripper joints to include in half posture if hands */
  if(hands)
  {
    gripperJoints.push_back("RHand");
    gripperJoints.push_back("RFinger11");
    gripperJoints.push_back("RFinger12");
    gripperJoints.push_back("RFinger13");
    gripperJoints.push_back("RFinger21");
    gripperJoints.push_back("RFinger22");
    gripperJoints.push_back("RFinger23");
    gripperJoints.push_back("RFinger31");
    gripperJoints.push_back("RFinger32");
    gripperJoints.push_back("RFinger33");
    gripperJoints.push_back("RFinger41");
    gripperJoints.push_back("RFinger42");
    gripperJoints.push_back("RFinger43");
    gripperJoints.push_back("RThumb1");
    gripperJoints.push_back("RThumb2");
    gripperJoints.push_back("LHand");
    gripperJoints.push_back("LFinger11");
    gripperJoints.push_back("LFinger12");
    gripperJoints.push_back("LFinger13");
    gripperJoints.push_back("LFinger21");
    gripperJoints.push_back("LFinger22");
    gripperJoints.push_back("LFinger23");
    gripperJoints.push_back("LFinger31");
    gripperJoints.push_back("LFinger32");
    gripperJoints.push_back("LFinger33");
    gripperJoints.push_back("LFinger41");
    gripperJoints.push_back("LFinger42");
    gripperJoints.push_back("LFinger43");
    gripperJoints.push_back("LThumb1");
    gripperJoints.push_back("LThumb2");
  }

  /* Default posture joint values in degrees */
  halfSitting["HeadYaw"] = {0};
  halfSitting["HeadPitch"] = {-30};
  halfSitting["LShoulderPitch"] = {65};
  halfSitting["LShoulderRoll"] = {3};
  halfSitting["RShoulderPitch"] = {65};
  halfSitting["RShoulderRoll"] = {-3};
  halfSitting["LElbowYaw"] = {-28};
  halfSitting["LElbowRoll"] = {-1.4};
  halfSitting["RElbowYaw"] = {28};
  halfSitting["RElbowRoll"] = {1.4};
  halfSitting["LWristYaw"] = {-46};
  halfSitting["RWristYaw"] = {46};
  halfSitting["HipRoll"] = {0};
  halfSitting["HipPitch"] = {-58};
  halfSitting["KneePitch"] = {28.9};
  if(hands)
  {
    for(const auto & gripJ : gripperJoints)
    {
      halfSitting[gripJ] = {0};
    }
  }

  /* Wheels bumpers */
  _devices.emplace_back(new mc_pepper::TouchSensor("FrontRight", "Tibia", sva::PTransformd::Identity()));
  _devices.emplace_back(new mc_pepper::TouchSensor("FrontLeft", "Tibia", sva::PTransformd::Identity()));
  _devices.emplace_back(new mc_pepper::TouchSensor("Back", "Tibia", sva::PTransformd::Identity()));

  /* Tactile sensors */
  _devices.emplace_back(new mc_pepper::TouchSensor("Head/Touch/Front", "Head", sva::PTransformd::Identity()));
  _devices.emplace_back(new mc_pepper::TouchSensor("Head/Touch/Rear", "Head", sva::PTransformd::Identity()));
  _devices.emplace_back(new mc_pepper::TouchSensor("Head/Touch/Middle", "Head", sva::PTransformd::Identity()));
  _devices.emplace_back(new mc_pepper::TouchSensor("RHand/Touch/Back", "r_wrist", sva::PTransformd::Identity()));
  _devices.emplace_back(new mc_pepper::TouchSensor("LHand/Touch/Back", "l_wrist", sva::PTransformd::Identity()));

  /* Audio device */
  _devices.emplace_back(new mc_pepper::Speaker("Speakers"));

  /* Visual display */
  _devices.emplace_back(new mc_pepper::VisualDisplay("Tablet", "tablet", sva::PTransformd::Identity()));

  /* 6DoF BodySensor */
  if(extraHardware)
  {
    _bodySensors.emplace_back("T265", "t265_pose", sva::PTransformd::Identity());
  }

  /* Grippers */
  if(hands)
  {
    // Module wide gripper configuration
    _gripperSafety = {0.15, 1.0};
    _grippers = {{"l_gripper", {"LHand"}, false}, {"r_gripper", {"RHand"}, false}};
  }

  /* Read URDF file */
  readUrdf("pepper", fixed, excludedLinks);

  /* Reference joint order */
  _ref_joint_order = {
      "KneePitch", // 0
      "HipPitch", // 1
      "HipRoll", // 2
      "HeadYaw", // 3
      "HeadPitch", // 4
      "LShoulderPitch", // 5
      "LShoulderRoll", // 6
      "LElbowYaw", // 7
      "LElbowRoll", // 8
      "LWristYaw", // 9
      "LHand", // 10
      "RShoulderPitch", // 11
      "RShoulderRoll", // 12
      "RElbowYaw", // 13
      "RElbowRoll", // 14
      "RWristYaw", // 15
      "RHand" // 16
  };

  /* Collision hulls */
  auto fileByBodyName = stdCollisionsFiles(mb);
  _convexHull = getConvexHull(fileByBodyName);

  /* Halfsit posture */
  _stance = halfSittingPose(mb);

  /* Critical self collisions */
  _minimalSelfCollisions = {mc_rbdyn::Collision("RBicep", "Head", 0.03, 0.01, 0.),
                            mc_rbdyn::Collision("RForeArm", "Head", 0.03, 0.01, 0.),
                            mc_rbdyn::Collision("r_wrist", "Head", 0.03, 0.01, 0.),
                            mc_rbdyn::Collision("LBicep", "Head", 0.03, 0.01, 0.),
                            mc_rbdyn::Collision("LForeArm", "Head", 0.03, 0.01, 0.),
                            mc_rbdyn::Collision("l_wrist", "Head", 0.03, 0.01, 0.),
                            mc_rbdyn::Collision("RForeArm", "torso", 0.03, 0.01, 0.),
                            mc_rbdyn::Collision("r_wrist", "torso", 0.03, 0.01, 0.),
                            mc_rbdyn::Collision("LForeArm", "torso", 0.03, 0.01, 0.),
                            mc_rbdyn::Collision("l_wrist", "torso", 0.03, 0.01, 0.),
                            mc_rbdyn::Collision("RForeArm", "Pelvis", 0.03, 0.01, 0.),
                            mc_rbdyn::Collision("r_wrist", "Pelvis", 0.03, 0.01, 0.),
                            mc_rbdyn::Collision("LForeArm", "Pelvis", 0.03, 0.01, 0.),
                            mc_rbdyn::Collision("l_wrist", "Pelvis", 0.03, 0.01, 0.),
                            mc_rbdyn::Collision("l_wrist", "r_wrist", 0.03, 0.01, 0.)};
  _commonSelfCollisions = _minimalSelfCollisions;

  /* Additional self collisions */
  if(hands)
  {
    _commonSelfCollisions.push_back(mc_rbdyn::Collision("Head", "LThumb2_link", 0.02, 0.01, 0.));
  }
}

void PepperRobotModule::readUrdf(const std::string & robotName,
                                 bool fixed,
                                 const std::vector<std::string> & filteredLinks)
{
  std::string urdfPath = path + "/urdf/" + robotName + ".urdf";
  if(!bfs::exists(urdfPath))
  {
    mc_rtc::log::error_and_throw("Could not open Pepper model at {}", urdfPath);
  }
  init(rbd::parsers::from_urdf_file(urdfPath, fixed, filteredLinks));
}

std::map<std::string, std::vector<double>> PepperRobotModule::halfSittingPose(const rbd::MultiBody & mb) const
{
  std::map<std::string, std::vector<double>> res;
  for(const auto & j : mb.joints())
  {
    if(halfSitting.count(j.name()))
    {
      res[j.name()] = halfSitting.at(j.name());
      for(auto & ji : res[j.name()])
      {
        ji = M_PI * ji / 180;
      }
    }
    else if(j.name() != "Root" && j.dof() > 0)
    {
      mc_rtc::log::warning("Joint {} has {} dof, but is not part of half sitting posture", j.name(), j.dof());
    }
  }
  return res;
}

std::map<std::string, std::pair<std::string, std::string>> PepperRobotModule::getConvexHull(
    const std::map<std::string, std::pair<std::string, std::string>> & files) const
{
  std::string convexPath = path + "/convex/";

  std::map<std::string, std::pair<std::string, std::string>> res;
  for(const auto & f : files)
  {
    bfs::path fpath = bfs::path(convexPath) / (f.second.second + "-ch.txt");
    if(bfs::exists(fpath))
    {
      res[f.first] = std::pair<std::string, std::string>(f.second.first, convexPath + f.second.second + "-ch.txt");
    }
  }
  return res;
}

std::map<std::string, std::pair<std::string, std::string>> PepperRobotModule::stdCollisionsFiles(
    const rbd::MultiBody & /*mb*/) const
{
  std::map<std::string, std::pair<std::string, std::string>> res;
  for(const auto & b : mb.bodies())
  {
    // Filter out virtual links without convex files
    if(std::find(std::begin(virtualLinks), std::end(virtualLinks), b.name()) == std::end(virtualLinks))
    {
      res[b.name()] = {b.name(), b.name()};
    }
  }
  return res;
}

void PepperRobotModule::forceLibraryLink(const mc_rbdyn::Robots & robots)
{
  mc_pepper::CoMRelativeBodyTask task("", robots, 0, 0, 0);
  mc_pepper::BoundedAccelerationConstr cnstr(0, 0, 0);
}

} // namespace mc_robots

extern "C"
{
  ROBOT_MODULE_API void MC_RTC_ROBOT_MODULE(std::vector<std::string> & names)
  {
    names = {"Pepper",
             "PepperFixed",
             "PepperNoHands",
             "PepperFixedNoHands",
             "PepperExtraHardware",
             "PepperFixedExtraHardware",
             "PepperExtraHardwareNoHands",
             "PepperFixedExtraHardwareNoHands"};
  }
  ROBOT_MODULE_API void destroy(mc_rbdyn::RobotModule * ptr)
  {
    delete ptr;
  }
  ROBOT_MODULE_API mc_rbdyn::RobotModule * create(const std::string & name)
  {
    ROBOT_MODULE_CHECK_VERSION("Pepper")
    if(name == "Pepper")
    {
      return new mc_robots::PepperRobotModule(false, true, false);
    }
    else if(name == "PepperFixed")
    {
      return new mc_robots::PepperRobotModule(true, true, false);
    }
    else if(name == "PepperNoHands")
    {
      return new mc_robots::PepperRobotModule(false, false, false);
    }
    else if(name == "PepperFixedNoHands")
    {
      return new mc_robots::PepperRobotModule(true, false, false);
    }
    else if(name == "PepperExtraHardware")
    {
      return new mc_robots::PepperRobotModule(false, true, true);
    }
    else if(name == "PepperFixedExtraHardware")
    {
      return new mc_robots::PepperRobotModule(true, true, true);
    }
    else if(name == "PepperExtraHardwareNoHands")
    {
      return new mc_robots::PepperRobotModule(false, false, true);
    }
    else if(name == "PepperFixedExtraHardwareNoHands")
    {
      return new mc_robots::PepperRobotModule(true, false, true);
    }
    else
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("Pepper module cannot create an object of type {}", name);
      return nullptr;
    }
  }
}
