#pragma once

#include <mc_rbdyn/RobotModuleMacros.h>
#include <mc_rbdyn_urdf/urdf.h>
#include <mc_rtc/logging.h>
#include <mc_robots/api.h>

namespace mc_robots
{

struct MC_ROBOTS_DLLAPI PepperRobotModule : public mc_rbdyn::RobotModule
{
public:
  PepperRobotModule(bool fixed, bool hands, bool extraHardware);

protected:
  void readUrdf(const std::string & robotName, bool fixed, const std::vector<std::string> & filteredLinks);
  std::map<std::string, std::vector<double>> halfSittingPose(const rbd::MultiBody & mb) const;
  std::vector< std::map<std::string, std::vector<double> > > nominalBounds(const mc_rbdyn_urdf::Limits & limits) const;
  std::map<std::string, std::pair<std::string, std::string>> stdCollisionsFiles(const rbd::MultiBody & mb) const;
  std::map<std::string, std::pair<std::string, std::string> > getConvexHull(const std::map<std::string, std::pair<std::string, std::string>> & files) const;
  // Ensure that mc_pepper::(tasks/constraints) libraries are loaded by mc_rtc when mc_pepper is loaded
  void forceLibraryLink();

public:
  std::vector<std::string> virtualLinks;
  std::map< std::string, std::vector<double> > halfSitting;
  mc_rbdyn_urdf::Limits limits;
  std::vector<std::string> excludedLinks;
  std::vector<std::string> gripperJoints;
};

} // namespace mc_robots

extern "C"
{
  ROBOT_MODULE_API void MC_RTC_ROBOT_MODULE(std::vector<std::string> & names)
  {
    names = {"Pepper", "PepperFixed", "PepperNoHands", "PepperFixedNoHands", "PepperExtraHardware",
             "PepperFixedExtraHardware", "PepperExtraHardwareNoHands", "PepperFixedExtraHardwareNoHands"};
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
