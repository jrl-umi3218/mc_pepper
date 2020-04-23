#include <VisualDisplay.h>

namespace mc_rbdyn
{

VisualDisplay::~VisualDisplay() = default;

VisualDisplay * VisualDisplay::clone() const
{
  return new VisualDisplay(*this);
}

} // namespace mc_rbdyn
