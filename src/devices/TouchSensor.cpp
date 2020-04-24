#include "TouchSensor.h"

namespace mc_pepper
{

TouchSensor::~TouchSensor() = default;

mc_rbdyn::SensorPtr TouchSensor::clone() const
{
  return mc_rbdyn::SensorPtr(new TouchSensor(*this));
}

} // namespace mc_pepper
