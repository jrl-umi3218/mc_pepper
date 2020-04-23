#include "Speaker.h"

namespace mc_rbdyn
{

Speaker::~Speaker() = default;

Speaker * Speaker::clone() const
{
  return new Speaker(*this);
}

} // namespace mc_rbdyn
