#include "Speaker.h"

namespace mc_pepper
{

Speaker::~Speaker() = default;

mc_rbdyn::DevicePtr Speaker::clone() const
{
  return mc_rbdyn::DevicePtr(new Speaker(*this));
}

} // namespace mc_pepper
