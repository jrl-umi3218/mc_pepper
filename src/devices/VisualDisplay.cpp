#include "VisualDisplay.h"

namespace mc_pepper
{

VisualDisplay::~VisualDisplay() = default;

mc_rbdyn::DevicePtr VisualDisplay::clone() const
{
  auto visualDisplay = new VisualDisplay(name_, parent_, X_p_s_);
  visualDisplay->url_=url_;
  visualDisplay->newURL_=newURL_;
  visualDisplay->succeed_=succeed_;
  visualDisplay->reset_=reset_;
  return mc_rbdyn::DevicePtr(visualDisplay);
}

} // namespace mc_pepper
