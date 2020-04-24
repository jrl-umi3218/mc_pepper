#include "VisualDisplay.h"

namespace mc_pepper
{

VisualDisplay::~VisualDisplay() = default;

mc_rbdyn::SensorPtr VisualDisplay::clone() const
{
  return mc_rbdyn::SensorPtr(new VisualDisplay(*this));
}

} // namespace mc_pepper
