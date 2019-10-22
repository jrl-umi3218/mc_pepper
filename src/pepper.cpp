#include "pepper.h"
#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;
#include <mc_rtc/logging.h>

#include <boost/algorithm/string.hpp>
#include <fstream>

#include "config.h"

namespace mc_robots
{
PepperRobotModule::PepperRobotModule(bool load_ffb)
 : RobotModule(PEPPER_DESCRIPTION_PATH, "pepper")
 {

  /* Path to surface descriptions */
  rsdf_dir = path + "/rsdf";

  /* Virtual links */
 	virtualLinks.push_back("base_link");
  virtualLinks.push_back("r_gripper");
  virtualLinks.push_back("l_gripper");
  virtualLinks.push_back("WheelFL_link");
  virtualLinks.push_back("WheelFR_link");
  virtualLinks.push_back("WheelB_link");
  virtualLinks.push_back("CameraTop_frame");
  virtualLinks.push_back("CameraTop_optical_frame");
  virtualLinks.push_back("RFinger41_link");
  virtualLinks.push_back("RFinger12_link");
  virtualLinks.push_back("RFinger31_link");
  virtualLinks.push_back("RFinger32_link");
  virtualLinks.push_back("RFinger22_link");
  virtualLinks.push_back("RFinger13_link");
  virtualLinks.push_back("RFinger21_link");
  virtualLinks.push_back("RFinger11_link");
  virtualLinks.push_back("RFinger43_link");
  virtualLinks.push_back("RFinger42_link");
  virtualLinks.push_back("RFinger33_link");
  virtualLinks.push_back("LFinger41_link");
  virtualLinks.push_back("LFinger12_link");
  virtualLinks.push_back("LFinger31_link");
  virtualLinks.push_back("LFinger32_link");
  virtualLinks.push_back("LFinger22_link");
  virtualLinks.push_back("LFinger13_link");
  virtualLinks.push_back("LFinger21_link");
  virtualLinks.push_back("LFinger11_link");
  virtualLinks.push_back("LFinger43_link");
  virtualLinks.push_back("LFinger42_link");
  virtualLinks.push_back("LFinger33_link");


  /* Init joint values in degrees */
 	halfSitting["HeadYaw"] = { 0.0 };
 	halfSitting["HeadPitch"] = { 30.0 };
 	halfSitting["LShoulderPitch"] = { 65.0 };
 	halfSitting["LShoulderRoll"] = { 3.0 };
 	halfSitting["RShoulderPitch"] = { 65.0 };
 	halfSitting["RShoulderRoll"] = { -3.0 };
 	halfSitting["LElbowYaw"] = { -28.0 };
 	halfSitting["LElbowRoll"] = { -1.0 };
  halfSitting["LHand"] = { 0.0 };
 	halfSitting["RElbowYaw"] = { 28.0 };
 	halfSitting["RElbowRoll"] = { 1.0 };
 	halfSitting["LWristYaw"] = { -46.0 };
 	halfSitting["RWristYaw"] = { 46.0 };
 	halfSitting["HipRoll"] = { 0.0 };
 	halfSitting["HipPitch"] = { -58.0 };
 	halfSitting["KneePitch"] = { 29.0 };
  halfSitting["RHand"] = { 0.0 };


  _ref_joint_order = {
 	"KneePitch",
 	"HipPitch",
 	"HipRoll",
 	"HeadYaw",
 	"HeadPitch",
 	"LShoulderPitch",
 	"LShoulderRoll",
 	"LElbowYaw",
 	"LElbowRoll",
 	"LWristYaw",
  "LHand",
 	"RShoulderPitch",
 	"RShoulderRoll",
 	"RElbowYaw",
 	"RElbowRoll",
 	"RWristYaw",
 	"RHand"
  };

  /* Read URDF file */
  if(!load_ffb)
  {
    readUrdf("pepper", filteredLinks);
  }
  else
  {
    std::vector<std::string> extraJoints {"Trans_Y", "Trans_X", "Rot_Z"};
    std::vector<double> mobileBase_velMax {0.35, 0.35, 1.0};
    _ref_joint_order.insert(_ref_joint_order.begin(), extraJoints.begin(), extraJoints.end());
    halfSitting["Trans_Y"] = {0};
    halfSitting["Trans_X"] = {0};
    halfSitting["Rot_Z"] = {0};
    readUrdf("pepper_ffb", {});
    // modify limits for extraJoints
    for(unsigned i=0;i<extraJoints.size();i++){
      auto jn = extraJoints[i];
      limits.lower[jn] = {-INFINITY};
      limits.upper[jn] = {INFINITY};
      limits.torque[jn] = {INFINITY};
      limits.velocity[jn] = {mobileBase_velMax[i]};
    }
  }

  auto fileByBodyName = stdCollisionsFiles(mb);
  _convexHull = getConvexHull(fileByBodyName);

  _bounds = nominalBounds(limits);
  _stance = halfSittingPose(mb);


  _minimalSelfCollisions = {
    mc_rbdyn::Collision("torso", "Head", 0.03, 0.01, 0.),
    mc_rbdyn::Collision("RBicep", "Head", 0.03, 0.01, 0.),
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
    mc_rbdyn::Collision("l_wrist", "r_wrist", 0.03, 0.01, 0.)
  };

  _commonSelfCollisions = _minimalSelfCollisions;
  _commonSelfCollisions.push_back(mc_rbdyn::Collision("Head", "LThumb2_link", 0.02, 0.01, 0.));
  _commonSelfCollisions.push_back(mc_rbdyn::Collision("Head", "LFinger23_link", 0.02, 0.01, 0.));
  _commonSelfCollisions.push_back(mc_rbdyn::Collision("Head", "RThumb2_link", 0.02, 0.01, 0.));
  _commonSelfCollisions.push_back(mc_rbdyn::Collision("Head", "RFinger23_link", 0.02, 0.01, 0.));
  _commonSelfCollisions.push_back(mc_rbdyn::Collision("torso", "LThumb2_link", 0.02, 0.01, 0.));
  _commonSelfCollisions.push_back(mc_rbdyn::Collision("torso", "LFinger23_link", 0.02, 0.01, 0.));
  _commonSelfCollisions.push_back(mc_rbdyn::Collision("torso", "RThumb2_link", 0.02, 0.01, 0.));
  _commonSelfCollisions.push_back(mc_rbdyn::Collision("torso", "RFinger23_link", 0.02, 0.01, 0.));
  _commonSelfCollisions.push_back(mc_rbdyn::Collision("Pelvis", "LThumb2_link", 0.02, 0.01, 0.));
  _commonSelfCollisions.push_back(mc_rbdyn::Collision("Pelvis", "LFinger23_link", 0.02, 0.01, 0.));
  _commonSelfCollisions.push_back(mc_rbdyn::Collision("Pelvis", "RThumb2_link", 0.02, 0.01, 0.));
  _commonSelfCollisions.push_back(mc_rbdyn::Collision("Pelvis", "RFinger23_link", 0.02, 0.01, 0.));

 }


 void PepperRobotModule::readUrdf(const std::string & robotName, const std::vector<std::string> & filteredLinks)
  {
    std::string urdfPath = path + "/urdf/" + robotName + ".urdf";
    std::ifstream ifs(urdfPath);
    if(ifs.is_open())
    {
      std::stringstream urdf;
      urdf << ifs.rdbuf();
      /* Consider robot as fixed base for now (even for ffb model) */
      mc_rbdyn_urdf::URDFParserResult res = mc_rbdyn_urdf::rbdyn_from_urdf(urdf.str(), true, filteredLinks);
      mb = res.mb;
      mbc = res.mbc;
      mbg = res.mbg;
      limits = res.limits;

      _visual = res.visual;
      _collisionTransforms = res.collision_tf;
    }
    else
    {
      LOG_ERROR("Could not open Pepper model at " << urdfPath)
      throw("Failed to open Pepper model");
    }
  }


  std::map<std::string, std::vector<double>> PepperRobotModule::halfSittingPose(const rbd::MultiBody & mb) const
  {
    std::map<std::string, std::vector<double>> res;
    for (const auto & j : mb.joints())
    {
      if(halfSitting.count(j.name()))
      {
        res[j.name()] = halfSitting.at(j.name());
        for (auto & ji : res[j.name()])
        {
          ji = M_PI*ji / 180;
        }
      }
      else if(j.name() != "Root" && j.dof() > 0)
      {
        LOG_WARNING("Joint " << j.name() << " has " << j.dof() << " dof, but is not part of half sitting posture.");
      }
    }
    return res;
  }


  std::map<std::string, std::pair<std::string, std::string> > PepperRobotModule::getConvexHull(const std::map<std::string, std::pair<std::string, std::string>> & files) const
  {
    std::string convexPath = path + "/convex/";

    std::map<std::string, std::pair<std::string, std::string> > res;
    for(const auto & f : files)
    {
      bfs::path fpath = bfs::path(convexPath)/(f.second.second+"-ch.txt");
      if (bfs::exists(fpath))
      {
       res[f.first] = std::pair<std::string, std::string>(f.second.first, convexPath + f.second.second + "-ch.txt");
      }
    }
    return res;
  }


  std::vector< std::map<std::string, std::vector<double> > > PepperRobotModule::nominalBounds(const mc_rbdyn_urdf::Limits & limits) const
  {
    std::vector< std::map<std::string, std::vector<double> > > res(0);
    res.push_back(limits.lower);
    res.push_back(limits.upper);
    {
      auto mvelocity = limits.velocity;
      for (auto & mv : mvelocity)
      {
        for (auto & mvi : mv.second)
        {
          mvi = -mvi;
        }
      }
      res.push_back(mvelocity);
    }
    res.push_back(limits.velocity);
    {
      auto mtorque = limits.torque;
      for (auto & mt : mtorque)
      {
        for (auto & mti : mt.second)
        {
          mti = -mti;
        }
      }
      res.push_back(mtorque);
    }
    res.push_back(limits.torque);
    return res;
  }

  std::map<std::string, std::pair<std::string, std::string>> PepperRobotModule::stdCollisionsFiles(const rbd::MultiBody &/*mb*/) const
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

  const std::map<std::string, std::pair<std::string, std::string> > & PepperRobotModule::convexHull() const
  {
    return _convexHull;
  }


  const std::vector< std::map<std::string, std::vector<double> > > & PepperRobotModule::bounds() const
  {
    return _bounds;
  }

  const std::map<std::string, std::vector<double> > & PepperRobotModule::stance() const
  {
    return _stance;
  }

}
