#include <TouchSensor.h>

namespace mc_rbdyn
{

TouchSensor::~TouchSensor() = default;

TouchSensor * TouchSensor::clone() const
{
  return new TouchSensor(*this);
}

} // namespace mc_rbdyn
