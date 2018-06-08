#include "pepper.h"
#include <boost/filesystem.hpp>
namespace bfs = boost::filesystem;
#include <mc_rtc/config.h>
#include <mc_rtc/logging.h>

#include <boost/algorithm/string.hpp>
#include <fstream>

namespace mc_rtc
{
  static std::string PEPPER_DESCRIPTION_PATH = PEPPER_DESCRIPTION_PATH_IN;
}

namespace mc_robots
{
PepperRobotModule::PepperRobotModule(bool load_ffb)
 : RobotModule(mc_rtc::PEPPER_DESCRIPTION_PATH, "pepper")
 {

  /* Path to surface descriptions */
  rsdf_dir = path + "/rsdf";

  /* Virtual links */
 	virtualLinks.push_back("base_link");
  virtualLinks.push_back("base_footprint");
  virtualLinks.push_back("SonarBack_frame");
  virtualLinks.push_back("CameraBottom_optical_frame");
  virtualLinks.push_back("CameraDepth_optical_frame");
  virtualLinks.push_back("LHandTouchBack_frame");
  virtualLinks.push_back("CameraBottom_optical_frame");
  virtualLinks.push_back("VerticalLeftLaser_frame");
  virtualLinks.push_back("SurroundingFrontLaser_device_frame");
  virtualLinks.push_back("BumperB_frame");
  virtualLinks.push_back("SurroundingLeftLaser_frame");
  virtualLinks.push_back("ShovelLaser_frame");
  virtualLinks.push_back("ImuTorsoAccelerometer_frame");
  virtualLinks.push_back("RSpeaker_frame");
  virtualLinks.push_back("SurroundingRightLaser_device_frame");
  virtualLinks.push_back("LSpeaker_frame");
  virtualLinks.push_back("RHandTouchBack_frame");
  virtualLinks.push_back("CameraTop_frame");
  virtualLinks.push_back("SurroundingLeftLaser_device_frame");
  virtualLinks.push_back("CameraDepth_frame");
  virtualLinks.push_back("SurroundingRightLaser_frame");
  virtualLinks.push_back("HeadTouchFront_frame");
  virtualLinks.push_back("VerticalRightLaser_frame");
  virtualLinks.push_back("SonarFront_frame");
  virtualLinks.push_back("BumperFL_frame");
  virtualLinks.push_back("CameraBottom_frame");
  virtualLinks.push_back("CameraTop_optical_frame");
  virtualLinks.push_back("ImuBaseAccelerometer_frame");
  virtualLinks.push_back("ImuTorsoGyrometer_frame");
  virtualLinks.push_back("Tablet_frame");
  virtualLinks.push_back("HeadTouchRear_frame");
  virtualLinks.push_back("SurroundingFrontLaser_frame");
  virtualLinks.push_back("ChestButton_frame");
  virtualLinks.push_back("HeadTouchMiddle_frame");
  virtualLinks.push_back("BumperFL_frame");

  /* Left hand fingers */
  /*filteredLinks.push_back("LFinger21_link");
  filteredLinks.push_back("LFinger22_link");
  filteredLinks.push_back("LFinger23_link");
  filteredLinks.push_back("LFinger11_link");
  filteredLinks.push_back("LFinger12_link");
  filteredLinks.push_back("LFinger13_link");
  filteredLinks.push_back("LFinger31_link");
  filteredLinks.push_back("LFinger32_link");
  filteredLinks.push_back("LFinger33_link");
  filteredLinks.push_back("LFinger41_link");
  filteredLinks.push_back("LFinger42_link");
  filteredLinks.push_back("LFinger43_link");
  filteredLinks.push_back("LThumb1_link");
  filteredLinks.push_back("LThumb2_link");*/

  /* Right hand fingers */
  /*filteredLinks.push_back("RFinger21_link");
  filteredLinks.push_back("RFinger22_link");
  filteredLinks.push_back("RFinger23_link");
  filteredLinks.push_back("RFinger11_link");
  filteredLinks.push_back("RFinger12_link");
  filteredLinks.push_back("RFinger13_link");
  filteredLinks.push_back("RFinger31_link");
  filteredLinks.push_back("RFinger32_link");
  filteredLinks.push_back("RFinger33_link");
  filteredLinks.push_back("RFinger41_link");
  filteredLinks.push_back("RFinger42_link");
  filteredLinks.push_back("RFinger43_link");
  filteredLinks.push_back("RThumb1_link");
  filteredLinks.push_back("RThumb2_link");*/

  /* Wheels */
  filteredLinks.push_back("WheelFL_link");
  filteredLinks.push_back("WheelB_link");
  filteredLinks.push_back("WheelFR_link");
  //filteredLinks.push_back("r_gripper");
  //filteredLinks.push_back("l_gripper");


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
 	halfSitting["HipPitch"] = { -59.5 };
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
    _ref_joint_order.insert(_ref_joint_order.begin(), extraJoints.begin(), extraJoints.end());
    readUrdf("pepper_ffb", {});
  }

  auto fileByBodyName = stdCollisionsFiles(mb);
  _convexHull = getConvexHull(fileByBodyName);

  _bounds = nominalBounds(limits);
  _stance = halfSittingPose(mb);

  
  _minimalSelfCollisions = {
    mc_rbdyn::Collision("torso", "Head", 0.02, 0.01, 0.),
    mc_rbdyn::Collision("RBicep", "Head", 0.02, 0.01, 0.),
    mc_rbdyn::Collision("RForeArm", "Head", 0.02, 0.01, 0.),
    mc_rbdyn::Collision("r_wrist", "Head", 0.02, 0.01, 0.),
    mc_rbdyn::Collision("LBicep", "Head", 0.02, 0.01, 0.),
    mc_rbdyn::Collision("LForeArm", "Head", 0.02, 0.01, 0.),
    mc_rbdyn::Collision("l_wrist", "Head", 0.02, 0.01, 0.),
    mc_rbdyn::Collision("RForeArm", "torso", 0.02, 0.01, 0.), 
    mc_rbdyn::Collision("r_wrist", "torso", 0.02, 0.01, 0.),
    mc_rbdyn::Collision("LForeArm", "torso", 0.02, 0.01, 0.),
    mc_rbdyn::Collision("l_wrist", "torso", 0.02, 0.01, 0.),
    mc_rbdyn::Collision("RForeArm", "Pelvis", 0.02, 0.01, 0.), 
    mc_rbdyn::Collision("r_wrist", "Pelvis", 0.02, 0.01, 0.),
    mc_rbdyn::Collision("LForeArm", "Pelvis", 0.02, 0.01, 0.),
    mc_rbdyn::Collision("l_wrist", "Pelvis", 0.02, 0.01, 0.),
    mc_rbdyn::Collision("l_wrist", "r_wrist", 0.02, 0.01, 0.) 
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
      /* Consider robot as fixed base for now with root at base_footprint */
      mc_rbdyn_urdf::URDFParserResult res = mc_rbdyn_urdf::rbdyn_from_urdf(urdf.str(), true, filteredLinks, true, "base_footprint");
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
