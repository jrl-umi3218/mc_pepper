/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/Sensor.h>

#include <vector>

namespace mc_pepper
{

/** This structure defines a speaker device, that is a device that can
 * play a sound or transform text into speach */
struct MC_RBDYN_DLLAPI Speaker : public mc_rbdyn::Sensor
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /** Default constructor, does not represent a valid body sensor */
  inline Speaker() : Speaker("", "", sva::PTransformd::Identity()) {}

  /** Constructor
   *
   * @param name Name of the sensor
   *
   * @param bodyName Name of the body to which the sensor is attached
   *
   * @param X_b_s Transformation from the parent body to the sensor
   *
   */
  inline Speaker(const std::string & name, const std::string & bodyName, const sva::PTransformd & X_b_s)
  : mc_rbdyn::Sensor(name, bodyName, X_b_s)
  {
    type_ = "Speaker";
  }

  ~Speaker() override;

  /** Get the sensor's parent body name */
  inline const std::string & parentBody() const
  {
    return Sensor::parent();
  }

  /** Return the transformation from the parent body to the sensor */
  inline const sva::PTransformd & X_b_s() const
  {
    return Sensor::X_p_s();
  }

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

  mc_rbdyn::SensorPtr clone() const override;

private:
  std::string text_ = "";
};

typedef std::vector<Speaker, Eigen::aligned_allocator<Speaker>> SpeakerVector;

} // namespace mc_pepper
