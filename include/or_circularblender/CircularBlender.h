#ifndef CIRCULARBLENDER_H_
#define CIRCULARBLENDER_H_

#include <openrave/openrave.h>
#include "CircularBlenderParameters.h"

namespace or_circularblender
{

class CircularBlender : public OpenRAVE::PlannerBase
{
public:
    CircularBlender(OpenRAVE::EnvironmentBasePtr penv);

    virtual bool InitPlan(OpenRAVE::RobotBasePtr robot,
                          PlannerParametersConstPtr params);
    virtual bool InitPlan(OpenRAVE::RobotBasePtr robot, std::istream &input);

    virtual OpenRAVE::PlannerStatus PlanPath(OpenRAVE::TrajectoryBasePtr ptraj);

    virtual PlannerParametersConstPtr GetParameters () const;

private:
    CircularBlenderParametersPtr parameters_;
};

} // or_circularblender

#endif // CIRCULARBLENDER_H_
