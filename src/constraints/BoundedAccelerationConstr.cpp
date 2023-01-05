#include "BoundedAccelerationConstr.h"

#include <mc_solver/ConstraintSetLoader.h>
#include <mc_solver/TasksQPSolver.h>

namespace details
{

BoundedAccelerationConstr::BoundedAccelerationConstr(unsigned int rIndex,
                                                     double maxAccTransXY,
                                                     double maxAccRotZ)
: mc_solver::GenInequalityConstraintRobot(rIndex),
  maxAccTransXY_(maxAccTransXY), maxAccRotZ_(maxAccRotZ)
  {
    A_.resize(3, 6);
    A_.setZero();
    A_(0, 2) = 1.0;
    A_(1, 3) = 1.0;
    A_(2, 4) = 1.0;

    L_.resize(3);
    L_.setZero();
    L_(0) = -maxAccRotZ;
    L_(1) = -maxAccTransXY;
    L_(2) = -maxAccTransXY;

    U_ = -L_.eval();
  }

} // namespace details

namespace mc_pepper
{
using Backend = mc_solver::QPSolver::Backend;

BoundedAccelerationConstr::BoundedAccelerationConstr(unsigned int rIndex, double maxAccTransXY, double maxAccRotZ)
: constr_(rIndex, maxAccTransXY, maxAccRotZ), rIndex_(rIndex)
{
}

void BoundedAccelerationConstr::addToSolverImpl(mc_solver::QPSolver & solver)
{
  switch(solver.backend())
  {
    case Backend::Tasks:
      {
        auto & solverT = tasks_solver(solver);
        constr_.addToSolver(solver.robots().mbs(), solverT.solver());
        solverT.updateConstrSize();
      }
      break;
    default:
      break;
  }
}

void BoundedAccelerationConstr::removeFromSolverImpl(mc_solver::QPSolver & solver)
{
  switch(solver.backend())
  {
    case Backend::Tasks:
      {
        auto & solverT = tasks_solver(solver);
        solverT.removeConstraint(&constr_);
        solverT.updateConstrSize();
      }
      break;
    default:
      break;
  }
}

/** This shows how a constraint can be registered with the ConstraintSetLoader */
static auto registered = mc_solver::ConstraintSetLoader::register_load_function(
    "pepper_boundedBaseAcceleration", // unique identifier
    [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) { // loading function
      return std::make_shared<BoundedAccelerationConstr>(robotIndexFromConfig(config, solver.robots(), "BoundedAccelerationConstr"), config("maxBaseTransAcc"), config("maxBaseRotAcc"));
    });

} // namespace mc_pepper
