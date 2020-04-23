/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/Sensor.h>

namespace mc_rbdyn
{

/** This structure defines a visual display device, that is a device that can
 * display images (e.g. tablet) */
struct MC_RBDYN_DLLAPI VisualDisplay : public Sensor
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /** Default constructor, does not represent a valid body sensor */
  inline VisualDisplay() : VisualDisplay("", "", sva::PTransformd::Identity()) {}

  /** Constructor
   *
   * @param name Name of the sensor
   *
   * @param bodyName Name of the body to which the sensor is attached
   *
   * @param X_b_s Transformation from the parent body to the sensor
   *
   */
  inline VisualDisplay(const std::string & name, const std::string & bodyName, const sva::PTransformd & X_b_s)
  : Sensor(name, bodyName, X_b_s)
  {
    type_ = "VisualDisplay";
  }

  ~VisualDisplay() override;

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

  /** Return the url of the image to be displayed */
  inline const std::string & display() const
  {
    return url_;
  }

  /** Set the url of the image to be displayed */
  inline void display(const std::string & url)
  {
   url_ = url;
  }

  VisualDisplay * clone() const override;

private:
  std::string url_ = "";
};

typedef std::vector<VisualDisplay, Eigen::aligned_allocator<VisualDisplay>> VisualDisplayVector;

} // namespace mc_rbdyn
