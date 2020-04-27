/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/Device.h>

#include <vector>

namespace mc_pepper
{

/** This structure defines a touch sensor, that is a sensor that can
 * detect a contact event (e.g. tactile, bumper switch etc.).
 * The reading of this sensor is either True (touched) or False (no touch) */
struct MC_RBDYN_DLLAPI TouchSensor : public mc_rbdyn::Device
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /** Default constructor, does not represent a valid touch sensor */
  inline TouchSensor() : TouchSensor("", "", sva::PTransformd::Identity()) {}

  /** Constructor
   *
   * @param name Name of the touch sensor
   *
   * @param bodyName Name of the body to which the touch sensor is attached
   *
   * @param X_b_s Transformation from the parent body to the touch sensor
   *
   */
  inline TouchSensor(const std::string & name, const std::string & bodyName, const sva::PTransformd & X_b_s)
  : mc_rbdyn::Device(name, bodyName, X_b_s)
  {
    type_ = "TouchSensor";
  }

  ~TouchSensor() override;

  /** Get the touch sensor's parent body name */
  inline const std::string & parentBody() const
  {
    return Device::parent();
  }

  /** Return the transformation from the parent body to the touch sensor */
  inline const sva::PTransformd & X_b_s() const
  {
    return Device::X_p_s();
  }

  /** Return the sensor's touch reading, false if not provided */
  inline const bool & touch() const
  {
    return touch_;
  }

  /** Set the sensor's position reading */
  inline void touch(const bool & touch)
  {
   touch_ = touch;
  }

  mc_rbdyn::DevicePtr clone() const override;

private:
  bool touch_ = false;
};

typedef std::vector<TouchSensor, Eigen::aligned_allocator<TouchSensor>> TouchSensorVector;

} // namespace mc_pepper
