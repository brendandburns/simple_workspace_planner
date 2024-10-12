// Copyright (c) 2015, TRACLabs, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#include <chrono>
#include <map>
#include <random>
#include <string>
#include <vector>

#include "kdl/chainiksolverpos_nr_jl.hpp"
#include "rclcpp/rclcpp.hpp"
#include "trac_ik/trac_ik.hpp"
#include "planner.h"
#include "logger.h"
#include "node_logger.h"


void test(
  const rclcpp::Node::SharedPtr node, double num_samples, std::string chain_start,
  std::string chain_end, double /*timeout*/, std::string urdf_xml)
{
  WorkspacePlanner planner(chain_start, chain_end, urdf_xml);

  std::vector<double> ll(6), ul(6);
  planner.limits(ll, ul);
  assert(planner.jointCount() == ll.size());
  assert(planner.jointCount() == ul.size());

  RCLCPP_INFO(node->get_logger(), "Using %d joints", planner.jointCount());

  std::vector<double> nominal(planner.jointCount());
  for (uint j = 0; j < nominal.size(); j++) {
    nominal[j] = (ll[j] + ul[j]) / 2.0;
  }

  std::vector<double> joints = {0,0,0,0,0,0};
  KDL::Frame pos, up;
  up.p = {0, 0, 1.65};

  RCLCPP_INFO_STREAM(
  node->get_logger(),
    "*** Testing KDL with cartesian position at zeros");

  planner.jointsToWorkspace(joints, pos);

  RCLCPP_INFO_STREAM(
  node->get_logger(), "*** Position is " << pos.p.x() << ", " << pos.p.y() << ", " << pos.p.z());

  planner.workspaceToJoints(nominal, up, joints);

  RCLCPP_INFO_STREAM(
  node->get_logger(), "*** Joints are " << joints[0] << ", " << joints[1] << ", " << joints[2] << ", " << joints[3] << ", " << joints[4] << ", " << joints[5] << ", ");

  // Create desired number of valid, random joint configurations
  std::vector<std::vector<double>> JointList;
  std::vector<double> q(planner.jointCount());

  std::random_device rd;
  std::mt19937 gen(rd());
  for (uint i = 0; i < num_samples; i++) {
    for (uint j = 0; j < ll.size(); j++) {
      std::uniform_real_distribution<double> dist(ll[j], ul[j]);
      q[j] = dist(gen);
    }
    JointList.push_back(q);
  }

  std::chrono::time_point<std::chrono::system_clock, std::chrono::duration<double>> start_time;
  std::chrono::duration<double> diff;

  std::vector<double> result(planner.jointCount());
  KDL::Frame end_effector_pose;

  double total_time = 0;
  uint success = 0;

  RCLCPP_INFO_STREAM(
    node->get_logger(), "*** Testing TRAC-IK with " << num_samples << " random samples");

  for (uint i = 0; i < num_samples; i++) {
    planner.jointsToWorkspace(JointList[i], end_effector_pose);

    start_time = std::chrono::system_clock::now();
    auto res = planner.workspaceToJoints(nominal, end_effector_pose, result);
    diff = std::chrono::system_clock::now() - start_time;
    total_time += diff.count();
    if (res) {
      success++;
    }

    if (static_cast<int>(static_cast<double>(i) / num_samples * 100) % 10 == 0) {
      RCLCPP_INFO_STREAM_THROTTLE(
        node->get_logger(),
        *(node->get_clock()),
        1.0,
        static_cast<int>((i) / num_samples * 100) << "% done");
    }
  }

  RCLCPP_INFO_STREAM(
    node->get_logger(),
    "TRAC-IK found " << success << " solutions (" << 100.0 * success / num_samples <<
      "%) with an average of " << total_time / num_samples <<
      " secs per sample");
  
  KDL::Frame startFrame, endFrame;
  std::vector<double> startJoints{0, 0, 0, 0, 0, 0};
  std::vector<std::vector<double>> path;

  startFrame.p = {0, 0, 1.65};
  endFrame.p = {0, 0, 1.35};

  planner.plan(startJoints, startFrame, endFrame, path);

  for (auto j : path)
  {
    std::stringstream of;
    of << j[0] << ", " << j[1] << ", " << j[2] << ", " << j[3] << ", " << j[4] << ", " << j[5] << std::endl;
    Logger::instance->info(of.str());

  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("workspace_planner");

  Logger::instance = std::unique_ptr<Logger>(new NodeLogger(node.get()));

  int num_samples;
  std::string chain_start, chain_end, urdf_xml;
  double timeout;

  node->declare_parameter<int>("num_samples", 1000);
  node->declare_parameter<double>("timeout", 0.005);
  node->declare_parameters<std::string>(
    std::string(),       // parameters are not namespaced
    std::map<std::string, std::string>{
    {"chain_start", std::string()},
    {"chain_end", std::string()},
    {"robot_description", std::string()},
  });

  node->get_parameter("num_samples", num_samples);
  node->get_parameter("timeout", timeout);
  node->get_parameter("chain_start", chain_start);
  node->get_parameter("chain_end", chain_end);
  node->get_parameter("robot_description", urdf_xml);

  if (chain_start.empty() || chain_end.empty()) {
    RCLCPP_FATAL(node->get_logger(), "Missing chain info in launch file");
    exit(-1);
  }

  if (num_samples < 1) {
    num_samples = 1;
  }

  test(node, num_samples, chain_start, chain_end, timeout, urdf_xml);

  // Useful when a script loops over multiple launch files testing different robot chains
  // std::vector<char *> commandVector;
  // commandVector.push_back((char*)"killall");
  // commandVector.push_back((char*)"-9");
  // commandVector.push_back((char*)"roslaunch");
  // commandVector.push_back(NULL);

  // char **command = &commandVector[0];
  // execvp(command[0],command);

  return 0;
}
