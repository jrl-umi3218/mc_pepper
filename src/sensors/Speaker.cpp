#include "Speaker.h"

namespace mc_pepper
{

Speaker::~Speaker() = default;

mc_rbdyn::SensorPtr Speaker::clone() const
{
  return mc_rbdyn::SensorPtr(new Speaker(*this));
}

} // namespace mc_pepper
