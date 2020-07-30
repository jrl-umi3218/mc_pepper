#include "TouchSensor.h"

namespace mc_pepper
{

TouchSensor::~TouchSensor() = default;

mc_rbdyn::DevicePtr TouchSensor::clone() const
{
  auto touchSensor = new TouchSensor(name_, parent_, X_p_s_);
  touchSensor->touch_=touch_;
  return mc_rbdyn::DevicePtr(touchSensor);
}

} // namespace mc_pepper
