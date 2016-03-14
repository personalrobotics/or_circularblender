# or_circularsmoother #
## An OpenRAVE Plugin for trajectory timing ##

This is an OpenRAVE wrapper to an algorithm for computing a time-optimal
trajectory to follow a path with bounded acceleration and velocity.

It is based on code provided by Tobias Kunz:
https://github.com/tobiaskunz/trajectories

which implements the technique described in the paper:
* Tobias Kunz and Mike Stilman, *Time-Optimal Trajectory Generation for Path Following with Bounded Acceleration and Velocity*, RSS 2008

## Parameters

This algorithm takes the following parameters (with the specified default values):

- `<integration_step>0.001</integration_step>`: The integration step size.
- `<interpolation_step>0.01</interpolation_step>`: The interpolation step size.
