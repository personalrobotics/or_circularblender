#include "CircularBlender.h"
#include "Path.h"
#include "Trajectory.h"

#include <boost/chrono.hpp>
#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>
#include <Eigen/Core>
#include <openrave/planningutils.h>

using boost::make_shared;
using OpenRAVE::EnvironmentBasePtr;
using OpenRAVE::RobotBasePtr;
using OpenRAVE::TrajectoryBasePtr;
using namespace or_circularblender;

namespace
{

const std::vector<OpenRAVE::dReal> empty;

void ConvertWaypoint(TrajectoryBasePtr const &output_traj,
                     Trajectory const &input_traj,
                     double t, double dt)
{
    OpenRAVE::ConfigurationSpecification const cspec
        = output_traj->GetConfigurationSpecification();

    // Query a particular waypoint on the trajectory.
    const Eigen::VectorXd q = input_traj.getPosition(t);
    const Eigen::VectorXd qd = input_traj.getVelocity(t);
    const size_t num_dof = q.size();

    // Verify that the DOF spec matches the expected size.
    BOOST_ASSERT((2*num_dof+1) == cspec.GetDOF());
    BOOST_ASSERT(q.size() == qd.size());

    // Convert each waypoint to OpenRAVE waypoint.
    std::vector<OpenRAVE::dReal> waypoint(cspec.GetDOF());
    for (size_t i_dof = 0; i_dof < num_dof; ++i_dof) {
        waypoint[i_dof] = q[i_dof];
        waypoint[i_dof + num_dof] = qd[i_dof];
    }
    waypoint[2 * num_dof] = dt;

    // Add the waypoint to the end of the OpenRAVE trajectory.
    output_traj->Insert(output_traj->GetNumWaypoints(), waypoint, false);
}

bool CheckCollision(CircularBlenderParametersPtr parameters,
                    TrajectoryBasePtr const &traj, int num_dof)
{
    std::vector<OpenRAVE::dReal> waypoint;

    traj->GetWaypoint(0, waypoint);
    std::vector<OpenRAVE::dReal> q_a(waypoint.begin(),
                                     waypoint.begin() + num_dof);
    std::vector<OpenRAVE::dReal> qd_a(waypoint.begin() + num_dof,
                                      waypoint.begin() + 2*num_dof);
    bool is_collided = parameters->CheckPathAllConstraints(
        q_a, q_a, empty, empty, 0, OpenRAVE::IT_OpenStart
    );

    for (int i = 1; i < traj->GetNumWaypoints() - 1; ++i)
    {
        traj->GetWaypoint(i, waypoint);
        std::vector<OpenRAVE::dReal> q_b(waypoint.begin(),
                                         waypoint.begin() + num_dof);
        std::vector<OpenRAVE::dReal> qd_b(waypoint.begin() + num_dof,
                                          waypoint.begin() + 2*num_dof);
        is_collided |= parameters->CheckPathAllConstraints(
            q_a, q_b, qd_a, qd_b, 0, OpenRAVE::IT_OpenStart
        );

        q_a = q_b;
        qd_a = qd_b;
    }

    return is_collided;
}

} // namespace

CircularBlender::CircularBlender(EnvironmentBasePtr penv)
    : OpenRAVE::PlannerBase(penv)
{
}

bool CircularBlender::InitPlan(RobotBasePtr robot,
                               PlannerParametersConstPtr params)
{
    parameters_ = boost::make_shared<CircularBlenderParameters>();
    parameters_->copy(params);
    return true;
}

bool CircularBlender::InitPlan(RobotBasePtr robot, std::istream &input)
{
    parameters_ = boost::make_shared<CircularBlenderParameters>();

    // Deserialize the PlannerParameters once. We only do this to set
    // _configurationspecification for the next step, so we put the stream back
    // where it was.
    int const marker = input.tellg();
    input >> *parameters_;
    input.seekg(marker);

    // The CheckPathAllConstraints function returns "true" (cast to an integer)
    // if the _checkpathvelocityconstraintsfn is NULL. This value defaults to
    // NULL and is not serialized in PlannerParameters. We re-initialize the
    // parameters with the default values.
    parameters_->SetConfigurationSpecification(
        GetEnv(), parameters_->_configurationspecification);

    // Restore any parameters that may have be overwritten by
    // SetConfigurationSpecification.
    input >> *parameters_;

    return true;
}

OpenRAVE::PlannerStatus CircularBlender::PlanPath(TrajectoryBasePtr traj)
{
    using OpenRAVE::ConfigurationSpecification;
    using OpenRAVE::KinBodyPtr;

    // Get references to environment and configuration specification.
    EnvironmentBasePtr const env = GetEnv();
    ConfigurationSpecification pos_cspec
        = parameters_->_configurationspecification;
    ConfigurationSpecification traj_cspec
        = traj->GetConfigurationSpecification();

    // TODO: How do we do this properly?
    // Change the interpolation of the trajectory to quadratic.
    BOOST_FOREACH(ConfigurationSpecification::Group &group,
                  pos_cspec._vgroups)
    {
        group.interpolation = "quadratic";
    }
    
    ConfigurationSpecification vel_cspec
        = pos_cspec.ConvertToVelocitySpecification();

    size_t num_dof = pos_cspec.GetDOF();
    RAVELOG_DEBUG("Detected %d DOFs.\n", num_dof);

    // Define data structures to hold waypoints and limits.
    Eigen::Map<Eigen::VectorXd> max_velocity(
        parameters_->_vConfigVelocityLimit.data(), num_dof);
    Eigen::Map<Eigen::VectorXd> max_acceleration(
        parameters_->_vConfigAccelerationLimit.data(), num_dof);
    RAVELOG_DEBUG("Setting velocity and acceleration limits.\n");

    // Iterate through waypoints and convert them to Eigen vectors.
    std::list<Eigen::VectorXd> waypoints;
    for (size_t i_waypoint = 0; i_waypoint < traj->GetNumWaypoints(); ++i_waypoint)
    {
        // Copy the waypoint to a std::vector.
        std::vector<OpenRAVE::dReal> input_waypoint;
        traj->GetWaypoint(i_waypoint, input_waypoint, pos_cspec);
        BOOST_ASSERT(input_waypoint.size() == num_dof);

        // Convert the waypoints to Eigen data structures.
        Eigen::VectorXd output_waypoint(num_dof);
        for (size_t i_dof = 0; i_dof < num_dof; ++i_dof)
            output_waypoint[i_dof] = input_waypoint[i_dof];
        waypoints.push_back(output_waypoint);
    }
    RAVELOG_DEBUG("Setting %d waypoints.\n", waypoints.size());

    // Perform circular blend trajectory creation.
    RAVELOG_DEBUG("Solving trajectory (step: %f, deviation: %f).\n",
                  parameters_->integration_step_, parameters_->max_deviation_);
    Trajectory trajectory(Path(waypoints,
                               parameters_->max_deviation_),
                          max_velocity, max_acceleration,
                          parameters_->integration_step_);
    if (!trajectory.isValid())
    {
        RAVELOG_WARN("Trajectory generation failed.\n");
        return OpenRAVE::PS_Failed;
    }
    const double duration = trajectory.getDuration();
    const double dt = parameters_->interpolation_step_;
    
    // Clear the trajectory to write in the output.
    traj->Remove(0, traj->GetNumWaypoints());
    BOOST_ASSERT(traj->GetNumWaypoints() == 0);

    // Convert back to an OpenRAVE trajectory.
    OpenRAVE::ConfigurationSpecification output_cspec = pos_cspec + vel_cspec;
    output_cspec.AddDeltaTimeGroup();

    RAVELOG_WARN("Resampling trajectory (step: %f, duration: %f).\n",
                  dt, duration);
    OpenRAVE::planningutils::ConvertTrajectorySpecification(traj, output_cspec);

    // Insert an interpolation of the solution as the output trajectory.
    // The delta-time for the first waypoint is always zero.
    double t = 0.0;
    for (; t < duration; t += dt)
        ConvertWaypoint(traj, trajectory, t, (t == 0.0) ? 0.0 : dt);

    // Manually insert the last waypoint when necessary.
    // Most of the time, the final iterated waypoint will not match the
    // duration exactly, but this catches the edge case where it does.
    if (t != duration)
        ConvertWaypoint(traj, trajectory, duration, duration - (t - dt));

    // Collision check the trajectory if requested.
    if (!parameters_->check_collision_)
        return OpenRAVE::PS_HasSolution;
    return (CheckCollision(parameters_, traj, num_dof)) ?
        OpenRAVE::PS_Failed : OpenRAVE::PS_HasSolution;
}

OpenRAVE::PlannerBase::PlannerParametersConstPtr
    CircularBlender::GetParameters () const
{
    return parameters_;
}
