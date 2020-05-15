#include "TouchSensor.h"

namespace mc_pepper
{

TouchSensor::~TouchSensor() = default;

mc_rbdyn::DevicePtr TouchSensor::clone() const
{
  return mc_rbdyn::DevicePtr(new TouchSensor(*this));
}

} // namespace mc_pepper
