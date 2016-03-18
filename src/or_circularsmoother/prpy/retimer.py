import copy
from prpy.planning.retimer import OpenRAVERetimer
from prpy.planning.base import PlanningMethod


class KunzCircularRetimer(OpenRAVERetimer):
    def __init__(self,
                 integration_timeout=0.1,
                 interpolation_timeout=0.1,
                 max_deviation=0.1,
                 **kwargs):
        super(KunzCircularRetimer, self).__init__(
            'KunzCircularSmoother', **kwargs)

        self.default_options.update({
            'integration_timeout': float(integration_timeout),
            'interpolation_timeout': float(interpolation_timeout),
            'max_deviation': float(max_deviation)
        })

    @PlanningMethod
    def RetimeTrajectory(self, robot, path, options=None, **kw_args):
        full_options = copy.deepcopy(self.default_options)
        if options is not None:
            full_options.update(copy.deepcopy(options))

        return super(KunzCircularRetimer, self).RetimeTrajectory(
            robot, path, options=full_options, **kw_args)
