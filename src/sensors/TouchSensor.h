/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/Sensor.h>

namespace mc_rbdyn
{

/** This structure defines a touch sensor, that is a sensor that can
 * detect a contact event (e.g. tactile, bumper switch etc.).
 * The reading of this sensor is either True (touched) or False (no touch) */
struct MC_RBDYN_DLLAPI TouchSensor : public Sensor
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /** Default constructor, does not represent a valid body sensor */
  inline TouchSensor() : TouchSensor("", "", sva::PTransformd::Identity()) {}

  /** Constructor
   *
   * @param name Name of the sensor
   *
   * @param bodyName Name of the body to which the sensor is attached
   *
   * @param X_b_s Transformation from the parent body to the sensor
   *
   */
  inline TouchSensor(const std::string & name, const std::string & bodyName, const sva::PTransformd & X_b_s)
  : Sensor(name, bodyName, X_b_s)
  {
    type_ = "TouchSensor";
  }

  ~TouchSensor() override;

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

  TouchSensor * clone() const override;

private:
  bool touch_ = false;
};

typedef std::vector<TouchSensor, Eigen::aligned_allocator<TouchSensor>> TouchSensorVector;

} // namespace mc_rbdyn
