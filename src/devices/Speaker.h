/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/Device.h>

#include <vector>

namespace mc_pepper
{

/** This structure defines a speaker device, that is a device that can
 * play a sound or transform text into speach */
struct MC_RBDYN_DLLAPI Speaker : public mc_rbdyn::Device
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /** Default constructor, does not represent a valid speaker */
  inline Speaker() : Speaker("") {}

  /** Constructor
   *
   * @param name Name of the speaker
   *
   * @param bodyName Name of the body to which the speaker is attached
   *
   * @param X_b_s Transformation from the parent body to the speaker
   *
   */
  inline Speaker(const std::string & name)
  : mc_rbdyn::Device(name)
  {
    type_ = "Speaker";
  }

  ~Speaker() override;

  /** Return the text to say and reset the initial state */
  inline const std::string say()
  {
    std::string out = "";
    std::swap(out, text_);
    return out;
  }

  /** Set the text to be said */
  inline void say(const std::string & text)
  {
   text_ = text;
  }

  /** Return true if text to say is not empty */
  inline bool hasSomethingToSay()
  {
   return text_!="";
  }

  mc_rbdyn::DevicePtr clone() const override;

private:
  std::string text_ = "";
};

typedef std::vector<Speaker, Eigen::aligned_allocator<Speaker>> SpeakerVector;

} // namespace mc_pepper
