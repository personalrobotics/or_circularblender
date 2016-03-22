# or_circularblender #
## An OpenRAVE Plugin for trajectory timing ##

This is an OpenRAVE wrapper to an algorithm for converting a linear joint space
path into a timed trajectory with bounded acceleration and velocity.  It does
this by adding circular blends between consecutive linear path segments, and
computing the time-optimal motion along the linear-circular-linear segments.

The resulting trajectory is then reinterpolated at a fixed discretization to
return a quadratic polynomial spline approximation of the solution.

It is based on code provided by Tobias Kunz:
https://github.com/tobiaskunz/trajectories

which implements the technique described in the paper:
* Tobias Kunz and Mike Stilman, *Time-Optimal Trajectory Generation for Path Following with Bounded Acceleration and Velocity*, RSS 2008

## Parameters

This algorithm takes the following parameters (with the specified default values):

- `<integration_step>0.001</integration_step>`: The integration step size.
- `<interpolation_step>0.01</interpolation_step>`: The interpolation step size.
