#include "VisualDisplay.h"

namespace mc_rbdyn
{

VisualDisplay::~VisualDisplay() = default;

SensorPtr VisualDisplay::clone() const
{
  return SensorPtr(new VisualDisplay(*this));
}

} // namespace mc_rbdyn
