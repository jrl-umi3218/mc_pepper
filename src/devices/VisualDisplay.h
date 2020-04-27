/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_rbdyn/Device.h>

#include <vector>

namespace mc_pepper
{

/** This structure defines a visual display device, that is a device that can
 * display images (e.g. tablet) */
struct MC_RBDYN_DLLAPI VisualDisplay : public mc_rbdyn::Device
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /** Default constructor, does not represent a valid visual display */
  inline VisualDisplay() : VisualDisplay("", "", sva::PTransformd::Identity()) {}

  /** Constructor
   *
   * @param name Name of the visual display
   *
   * @param bodyName Name of the body to which the visual display is attached
   *
   * @param X_b_s Transformation from the parent body to the visual display
   *
   */
  inline VisualDisplay(const std::string & name, const std::string & bodyName, const sva::PTransformd & X_b_s)
  : mc_rbdyn::Device(name, bodyName, X_b_s)
  {
    type_ = "VisualDisplay";
  }

  ~VisualDisplay() override;

  /** Get the display's parent body name */
  inline const std::string & parentBody() const
  {
    return Device::parent();
  }

  /** Return the transformation from the parent body to the display */
  inline const sva::PTransformd & X_b_s() const
  {
    return Device::X_p_s();
  }

  /** Return the url of the image to be displayed */
  inline std::string & display()
  {
    newURL_ = false;
    return url_;
  }

  /** Set the url of the image to be displayed */
  inline void display(const std::string & url)
  {
   url_ = url;
   newURL_ = true;
  }

  /** Return true if the URL of image to display was updated */
  inline const bool & newURL() const
  {
    return newURL_;
  }

  /** Return if succeed to show image */
  inline bool & succeed()
  {
    return succeed_;
  }

  /** Set state of image display */
  inline void succeed(const bool & state)
  {
   succeed_ = state;
  }

  mc_rbdyn::DevicePtr clone() const override;

private:
  std::string url_ = "";
  bool newURL_ = false;
  bool succeed_ = false;
};

typedef std::vector<VisualDisplay, Eigen::aligned_allocator<VisualDisplay>> VisualDisplayVector;

} // namespace mc_pepper
