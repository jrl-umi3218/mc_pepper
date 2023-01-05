#pragma once

#include <mc_solver/GenInequalityConstraint.h>
#include <mc_solver/ConstraintSet.h>

namespace details
{

struct BoundedAccelerationConstr : public mc_solver::GenInequalityConstraintRobot{

  BoundedAccelerationConstr(unsigned int rIndex, double maxAccTransXY, double maxAccRotZ);

  inline int maxGenInEq() const override { return 3; }

  inline std::string nameGenInEq() const override { return "BoundedAccelerationConstr"; }

  inline const Eigen::VectorXd & LowerGenInEq() const override { return L_; }

  inline const Eigen::VectorXd & UpperGenInEq() const override { return U_; }

  inline const Eigen::MatrixXd & A() const override { return A_; }

  inline void compute() override {}

  private:
    // Max translation_XY acceleration
    double maxAccTransXY_;
    // Max rotation_Z acceleration
    double maxAccRotZ_;

    // L_ <= A*x <= U_
    Eigen::MatrixXd A_;
    Eigen::VectorXd L_;
    Eigen::VectorXd U_;
  };

} // namespace details

namespace mc_pepper
{

struct BoundedAccelerationConstr : public mc_solver::ConstraintSet
{
  BoundedAccelerationConstr(unsigned int rIndex, double maxAccTransXY, double maxAccRotZ);

 protected:
  void addToSolverImpl(mc_solver::QPSolver & solver) override;
  virtual void removeFromSolverImpl(mc_solver::QPSolver & solver) override;
private:
  details::BoundedAccelerationConstr constr_;
  unsigned rIndex_ = 0;
};

} // namespace mc_pepper
