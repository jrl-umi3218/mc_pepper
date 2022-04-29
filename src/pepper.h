#pragma once

#include <mc_robots/api.h>

#include <mc_rbdyn/RobotModuleMacros.h>
#include <mc_rbdyn/Robots.h>

#include <mc_rtc/logging.h>

namespace mc_robots
{

struct MC_ROBOTS_DLLAPI PepperRobotModule : public mc_rbdyn::RobotModule
{
public:
  PepperRobotModule(bool fixed, bool hands, bool extraHardware);

protected:
  void readUrdf(const std::string & robotName, bool fixed, const std::vector<std::string> & filteredLinks);
  std::map<std::string, std::vector<double>> halfSittingPose(const rbd::MultiBody & mb) const;
  std::map<std::string, std::pair<std::string, std::string>> stdCollisionsFiles(const rbd::MultiBody & mb) const;
  std::map<std::string, std::pair<std::string, std::string>> getConvexHull(
      const std::map<std::string, std::pair<std::string, std::string>> & files) const;
  // Ensure that mc_pepper::(tasks/constraints) libraries are loaded by mc_rtc when mc_pepper is loaded
  void forceLibraryLink(const mc_rbdyn::Robots & robots);

public:
  std::vector<std::string> virtualLinks;
  std::map<std::string, std::vector<double>> halfSitting;
  std::vector<std::string> excludedLinks;
  std::vector<std::string> gripperJoints;
};

} // namespace mc_robots
