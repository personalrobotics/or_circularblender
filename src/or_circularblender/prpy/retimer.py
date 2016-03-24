from copy import deepcopy
from prpy.planning.retimer import OpenRAVERetimer
from prpy.planning.base import PlanningMethod


class KunzCircularBlender(OpenRAVERetimer):
    def __init__(self,
                 check_collision=False,
                 integration_step=0.1,
                 interpolation_step=0.1,
                 max_deviation=0.1,
                 **kwargs):
        super(KunzCircularBlender, self).__init__(
            'KunzCircularBlender', **kwargs)

        self.default_options.update({
            'check_collision': bool(check_collision),
            'integration_step': float(integration_step),
            'interpolation_step': float(interpolation_step),
            'max_deviation': float(max_deviation)
        })

    @PlanningMethod
    def RetimeTrajectory(self, robot, path, options=None, **kw_args):
        new_options = deepcopy(options) if options else dict()
        return super(KunzCircularBlender, self).RetimeTrajectory(
            robot, path, options=new_options, **kw_args)
