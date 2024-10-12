#include <chrono>
#include <map>
#include <random>
#include <string>
#include <vector>

#include <kdl/trajectory.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/trajectory_stationary.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/trajectory_composite.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/path_roundedcomposite.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <rclcpp/rclcpp.hpp>
#include <trac_ik/trac_ik.hpp>

#include "planner.h"
#include "logger.h"

WorkspacePlanner::WorkspacePlanner(const std::string& first, const std::string& last, const std::string& urdf)
{
  double eps = 1e-5;
  double timeout = 0.005;

  ik = std::unique_ptr<TRAC_IK::TRAC_IK>(new TRAC_IK::TRAC_IK(first, last, urdf, timeout, eps));
  if (!ik->getKDLChain(ch)) {
    Logger::instance->error("Chain is invalid");
    return;
  }
  fk = std::unique_ptr<KDL::ChainFkSolverPos_recursive>(new KDL::ChainFkSolverPos_recursive(ch));
}

WorkspacePlanner::~WorkspacePlanner() { }

void WorkspacePlanner::jointsToWorkspace(const std::vector<double>& joints, KDL::Frame& frame)
{
  if (ch.getNrOfJoints() != joints.size())
  {
    Logger::instance->error("Wrong number of joints");
    return;
  }
  KDL::JntArray input(ch.getNrOfJoints());
  for (size_t i = 0; i < joints.size(); i++)
  {
    input(i) = joints[i];
  }
  fk->JntToCart(input, frame);
}

bool WorkspacePlanner::workspaceToJoints(const std::vector<double>& starting, const KDL::Frame& frame, std::vector<double>& joints)
{
  if (ch.getNrOfJoints() != joints.size())
  {
    Logger::instance->error("Wrong number of joints");
    return false;
  }

  KDL::JntArray current(starting.size());
  for (size_t i = 0; i < starting.size(); i++)
  {
    current(i) = starting[i];
  }

  KDL::JntArray result(ch.getNrOfJoints());
  auto res = ik->CartToJnt(current, frame, result);
  if (res < 0)
  {
    return false;
  }

  for (size_t i = 0; i < joints.size(); i++)
  {
    joints[i] = result(i);
  }
  return true;
}

unsigned int WorkspacePlanner::jointCount()
{
  return ch.getNrOfJoints();
}

void WorkspacePlanner::limits(std::vector<double>& lower_limit, std::vector<double>& upper_limit)
{
  KDL::JntArray ll, ul;
  ik->getKDLLimits(ll, ul);

  for (size_t i = 0; i < lower_limit.size(); i++)
  {
    lower_limit[i] = ll(i);
    upper_limit[i] = ul(i);
  }
}

void WorkspacePlanner::nominal(std::vector<double>& joints)
{
  KDL::JntArray ll, ul;
  ik->getKDLLimits(ll, ul);

  for (size_t i = 0; i < joints.size(); i++)
  {
    joints[i] = (ll(i) + ul(i)) / 2.0;
  }
}

bool WorkspacePlanner::plan(const std::vector<double>& jointStart, const KDL::Frame &start, const KDL::Frame &end, std::vector<std::vector<double>>& out)
{
  std::vector<double> startJoints(jointCount());
  std::vector<double> joints = jointStart;

  if (!workspaceToJoints(joints, start, startJoints))
  {
    Logger::instance->warning("Start position is invalid.");
    return false;
  }

  KDL::Path_RoundedComposite* path = new KDL::Path_RoundedComposite(0.2,0.01,new KDL::RotationalInterpolation_SingleAxis());
  path->Add(start);
  path->Add(end);
  path->Finish();


	KDL::VelocityProfile* velpref = new KDL::VelocityProfile_Trap(0.5,0.1);
	velpref->SetProfile(0,path->PathLength());  
	KDL::Trajectory* traject = new KDL::Trajectory_Segment(path, velpref);


	KDL::Trajectory_Composite* ctraject = new KDL::Trajectory_Composite();
	ctraject->Add(traject);
	ctraject->Add(new KDL::Trajectory_Stationary(1.0, end));

  double dt=0.1;
	for (double t=0.0; t <= traject->Duration(); t+= dt) {
		auto current_pose = traject->Pos(t);

    if (!workspaceToJoints(startJoints, current_pose, joints)) {
      std::stringstream msg;
      msg << "Failed to find joint configuration for " << current_pose.p.x() << ", " << current_pose.p.y() << ", " << current_pose.p.z();
      Logger::instance->error(msg.str());
      return false;
    }
    out.push_back(joints);
    startJoints = joints;
	}

  return true;
}

