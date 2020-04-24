#include "TouchSensor.h"

namespace mc_rbdyn
{

TouchSensor::~TouchSensor() = default;

SensorPtr TouchSensor::clone() const
{
  return SensorPtr(new TouchSensor(*this));
}

} // namespace mc_rbdyn
