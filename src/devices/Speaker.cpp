#include "Speaker.h"

namespace mc_pepper
{

Speaker::~Speaker() = default;

mc_rbdyn::DevicePtr Speaker::clone() const
{
  auto speaker = new Speaker(name_);
  speaker->text_=text_;
  return mc_rbdyn::DevicePtr(speaker);
}

} // namespace mc_pepper
