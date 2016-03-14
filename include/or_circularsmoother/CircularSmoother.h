#ifndef CIRCULARSMOOTHER_H_
#define CIRCULARSMOOTHER_H_
#include <openrave/openrave.h>
#include "DynamicPath.h"
#include "CircularSmootherParameters.h"

namespace or_circularsmoother {

class CircularSmoother : public OpenRAVE::PlannerBase {
public:
    CircularSmoother(OpenRAVE::EnvironmentBasePtr penv);

    virtual bool InitPlan(OpenRAVE::RobotBasePtr robot,
                          PlannerParametersConstPtr params);
    virtual bool InitPlan(OpenRAVE::RobotBasePtr robot, std::istream &input);

    virtual OpenRAVE::PlannerStatus PlanPath(OpenRAVE::TrajectoryBasePtr ptraj);

    virtual PlannerParametersConstPtr GetParameters () const;

private:
    CircularSmootherParametersPtr parameters_;
};


}

#endif
