#include "Speaker.h"

namespace mc_rbdyn
{

Speaker::~Speaker() = default;

SensorPtr Speaker::clone() const
{
  return SensorPtr(new Speaker(*this));
}

} // namespace mc_rbdyn
