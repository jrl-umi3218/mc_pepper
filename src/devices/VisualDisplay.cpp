#include "VisualDisplay.h"

namespace mc_pepper
{

VisualDisplay::~VisualDisplay() = default;

mc_rbdyn::DevicePtr VisualDisplay::clone() const
{
  return mc_rbdyn::DevicePtr(new VisualDisplay(*this));
}

} // namespace mc_pepper
