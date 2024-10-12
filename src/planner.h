#ifndef __PLANNER_H__
#define __PLANNER_H__

#include <memory>
#include <vector>

#include "kdl/chainiksolverpos_nr_jl.hpp"
#include "trac_ik/trac_ik.hpp"

class WorkspacePlanner
{
    private:
        KDL::Chain ch;
        std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk;
        std::unique_ptr<TRAC_IK::TRAC_IK> ik;
    
    public:
        WorkspacePlanner(const std::string& start, const std::string& finish, const std::string& urdf);
        ~WorkspacePlanner();

        void jointsToWorkspace(const std::vector<double> &joints, KDL::Frame &frame);
        bool workspaceToJoints(const std::vector<double> &starting, const KDL::Frame &frame, std::vector<double> &joints);
        bool plan(const std::vector<double> &startJoints, const KDL::Frame &start, const KDL::Frame &end, std::vector<std::vector<double>>& path);

        unsigned int jointCount();
        void limits(std::vector<double>& lower_limit, std::vector<double>& upper_limit);
        void nominal(std::vector<double>& angles);
};

#endif // __PLANNER_H__