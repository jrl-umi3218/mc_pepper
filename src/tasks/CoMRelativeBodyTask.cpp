#include "CoMRelativeBodyTask.h"

#include <mc_rtc/gui/Point3D.h>
#include <mc_rtc/gui/ArrayInput.h>
#include <mc_tasks/MetaTaskLoader.h>

namespace details
{

CoMRelativeBodyTask::CoMRelativeBodyTask(const mc_rbdyn::Robots & robots,
                                         unsigned int robotIndex,
                                         const std::string & body,
                                         const Eigen::Vector3d & target)
: robots_(robots), robotIndex_(robotIndex), comJac_(robots.robot(robotIndex).mb()), bodyJac_(robots.robot(robotIndex).mb(), body), target_(target)
{
  const auto & robot = robots_.robot(robotIndex_);
  bIndex_ = robot.bodyIndexByName(body);
  eval_.setZero(3);
  speed_.setZero(3);
  normalAcc_.setZero(3);
  jacMat_.setZero(3, robot.mb().nrDof());
}

int CoMRelativeBodyTask::dim()
{
  return 3;
}

void CoMRelativeBodyTask::update(const std::vector<rbd::MultiBody> &,
                                 const std::vector<rbd::MultiBodyConfig> &,
                                 const tasks::qp::SolverData &)
{
  const auto & robot_ = robots_.robot(robotIndex_);
  eval_ = -target_ + robot_.com() - robot_.mbc().bodyPosW[bIndex_].translation();
  speed_ = robot_.mbc().bodyVelW[bIndex_].linear() - robot_.comVelocity();
  normalAcc_ = bodyJac_.normalAcceleration(robot_.mb(), robot_.mbc()).linear()
               - comJac_.normalAcceleration(robot_.mb(), robot_.mbc());

  const auto & bJac = bodyJac_.jacobian(robot_.mb(), robot_.mbc()).block(3, 0, 3, bodyJac_.dof());
  bodyJac_.fullJacobian(robot_.mb(), bJac, jacMat_);
  jacMat_-= comJac_.jacobian(robot_.mb(), robot_.mbc());
}

const Eigen::MatrixXd & CoMRelativeBodyTask::jac() const
{
  return jacMat_;
}

const Eigen::VectorXd & CoMRelativeBodyTask::eval() const
{
  return eval_;
}

const Eigen::VectorXd & CoMRelativeBodyTask::speed() const
{
  return speed_;
}

const Eigen::VectorXd & CoMRelativeBodyTask::normalAcc() const
{
  return normalAcc_;
}

void CoMRelativeBodyTask::target(const Eigen::Vector3d & target)
{
  target_ = target;
}

const Eigen::Vector3d & CoMRelativeBodyTask::target() const
{
  return target_;
}

const mc_rbdyn::Robot & CoMRelativeBodyTask::robot() const
{
  return robots_.robot(robotIndex_);
}

unsigned int CoMRelativeBodyTask::bIndex() const
{
  return bIndex_;
}

} // namespace details

namespace mc_pepper
{

static inline auto tasks_error(mc_rtc::void_ptr & ptr)
{
  return static_cast<details::CoMRelativeBodyTask*>(ptr.get());
}

static inline const auto tasks_error(const mc_rtc::void_ptr & ptr)
{
  return static_cast<details::CoMRelativeBodyTask*>(ptr.get());
}

CoMRelativeBodyTask::CoMRelativeBodyTask(const std::string & body, const mc_rbdyn::Robots & robots, unsigned int robotIndex, double stiffness, double weight)
: mc_tasks::TrajectoryTaskGeneric(robots, robotIndex, stiffness, weight), body_(body), bIndex_(robots.robot(robotIndex).bodyIndexByName(body))
{
  const auto & robot = robots.robot(robotIndex);
  Eigen::Vector3d init = robot.bodyPosW(body).translation() - robot.com();
  switch(backend_)
  {
    case Backend::Tasks:
      finalize<Backend::Tasks, details::CoMRelativeBodyTask>(robots, robotIndex, body, init);
      break;
    default:
      mc_rtc::log::error_and_throw("[CoMRelativeBodyTask] Not implemented for solver backend: {}", backend_);
  }
  type_ = "com_relative_body";
  name_ = "com_relative_" + body;
}

void CoMRelativeBodyTask::target(const Eigen::Vector3d & pos)
{
  switch(backend_)
  {
    case Backend::Tasks:
      tasks_error(errorT)->target(pos);
      break;
    default:
      break;
  }
}

const Eigen::Vector3d & CoMRelativeBodyTask::target() const
{
  switch(backend_)
  {
    case Backend::Tasks:
      return tasks_error(errorT)->target();
      break;
    default:
      mc_rtc::log::error_and_throw("[CoMRelativeBodyTask] Not implemented for solver backend: {}", backend_);
  }
}

const Eigen::Vector3d CoMRelativeBodyTask::actual() const
{
  switch(backend_)
  {
    case Backend::Tasks:
      return tasks_error(errorT)->robot().com();
    default:
      mc_rtc::log::error_and_throw("[CoMRelativeBodyTask] Not implemented for solver backend: {}", backend_);
  }
}

void CoMRelativeBodyTask::reset()
{
  switch(backend_)
  {
    case Backend::Tasks:
      {
      const auto & err = tasks_error(errorT);
      const auto & robot = err->robot();
      target(robots.robot(rIndex).mbc().bodyPosW[bIndex_].translation() + robot.com());
      }
      break;
    default:
      break;
  }
}

void CoMRelativeBodyTask::addToLogger(mc_rtc::Logger & logger)
{
  Base::addToLogger(logger);
  logger.addLogEntry(name_ + "_eval", [this]() { return this->eval(); });
  logger.addLogEntry(name_ + "_body_pos", [this]() -> const Eigen::Vector3d & { return robots.robot(rIndex).mbc().bodyPosW[bIndex_].translation(); });
  logger.addLogEntry(name_ + "_com", [this]() { return this->actual(); });
  logger.addLogEntry(name_ + "_comTarget", [this]() { return this->target(); });
}

void CoMRelativeBodyTask::removeFromLogger(mc_rtc::Logger & logger)
{
  Base::removeFromLogger(logger);
  logger.removeLogEntry(name_ + "_eval");
  logger.removeLogEntry(name_ + "_body_pos");
  logger.removeLogEntry(name_ + "_com");
  logger.removeLogEntry(name_ + "_comTarget");
}

void CoMRelativeBodyTask::addToGUI(mc_rtc::gui::StateBuilder & gui)
{
  Base::addToGUI(gui);
  gui.addElement({"Tasks", name_}, mc_rtc::gui::ArrayInput("relPos", [this]() { return this->target(); },
                                      [this](const Eigen::VectorXd & pos) { this->target(pos); }),
                                   mc_rtc::gui::Point3D("com", [this]() { return this->actual(); }),
                                   mc_rtc::gui::Point3D("target", mc_rtc::gui::PointConfig({0., 1., 0.}, 0.03),
                                      [this]() { return Eigen::Vector3d(this->target() + robots.robot(rIndex).mbc().bodyPosW[bIndex_].translation() ); }));
}

/** This shows how a MetaTask can be registered with the mc_rtc loader */
static auto registered = mc_tasks::MetaTaskLoader::register_load_function(
  "com_relative_body", // unique type identifier
  [](mc_solver::QPSolver & solver, const mc_rtc::Configuration & config) { // loading function
    auto robotIndex = robotIndexFromConfig(config, solver.robots(), "CoMRelativeBodyTask");
    auto t = std::make_shared<CoMRelativeBodyTask>(config("body"), solver.robots(), robotIndex, config("stiffness"), config("weight"));
    if(config.has("target"))
    {
      t->target(config("target"));
    }
    t->load(solver, config);
    return t;
  });

} // namespace mc_pepper
