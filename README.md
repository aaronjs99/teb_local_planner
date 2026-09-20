teb_local_planner ROS Package
=============================

The teb_local_planner package implements a plugin to the base_local_planner of the 2D navigation stack. 
The underlying method called Timed Elastic Band locally optimizes the robot's trajectory with respect to trajectory execution time, 
separation from obstacles and compliance with kinodynamic constraints at runtime.

Refer to http://wiki.ros.org/teb_local_planner for more information and tutorials.

Build status of the *melodic-devel* branch:
- ROS Buildfarm (Melodic): [![Melodic Status](http://build.ros.org/buildStatus/icon?job=Mdev__teb_local_planner__ubuntu_bionic_amd64)](http://build.ros.org/job/Mdev__teb_local_planner__ubuntu_bionic_amd64/)


## GRANDE source dependency

GRANDE pins this fork to an exact commit through `dependencies.rosinstall`.
The package name remains `teb_local_planner`; use this source build for MARINER's
route, footprint and terminal-observation interfaces. Install `libnlopt-dev`
through the system package manager. NLopt is a private implementation dependency
of the planner library, not a developer-specific include or link path exported
to its consumers. Simulation and hardware use this same planner implementation.

## Nonholonomic motion model

`PoseSE2::longitudinalDistanceTo` supplies the signed displacement used by
velocity, acceleration, boundary-acceleration and obstacle-speed costs, and by
command/profile extraction. Differential surge/yaw planning always uses the
midpoint heading and chord-to-arc factor of a held unicycle twist, matching the
control rollout. For car-like planning, `exact_arc_length` selects that calculation;
otherwise displacement is projected onto the initial body heading. Both preserve
the linear small-displacement limit. Holonomic velocity calculations and
turn-radius constraints retain their separate definitions. Numerical Jacobians
use these same cost functions. Command continuity and nominal station/obstacle
checks remain independent; they do not certify actual tracking or stopping.

For MARINER route contracts, ordered progress belongs to MARINER's route
tracker. TEB reports the terminal pose, heading, and stopped-motion decision
from one control cycle without requiring retained global-plan via-points to be
empty. Standalone TEB goals retain the configured complete_global_plan
behavior. In a persistent session, TEB continues publishing its accepted
feedback command after arrival so the retained pose remains under closed-loop
control.

Differential-drive command extraction uses the configured timed preview, averaging
surge and yaw controls over the represented duration. It stops before a direction
reversal or a translation-to-rotation transition; long pivots and explicit route
turns remain separate. This lets a short initial alignment turn flow into forward
or reverse translation instead of restarting that pivot every planning cycle.
The shaped command still undergoes the existing swept-arc feasibility check;
the averaged command is not assumed to reproduce the entire piecewise path.

## Citing the Software

*Since a lot of time and effort has gone into the development, please cite at least one of the following publications if you are using the planner for your own research:*

- C. Rösmann, F. Hoffmann and T. Bertram: Integrated online trajectory planning and optimization in distinctive topologies, Robotics and Autonomous Systems, Vol. 88, 2017, pp. 142–153.
- C. Rösmann, W. Feiten, T. Wösch, F. Hoffmann and T. Bertram: Trajectory modification considering dynamic constraints of autonomous robots. Proc. 7th German Conference on Robotics, Germany, Munich, May 2012, pp 74–79.
- C. Rösmann, W. Feiten, T. Wösch, F. Hoffmann and T. Bertram: Efficient trajectory optimization using a sparse model. Proc. IEEE European Conference on Mobile Robots, Spain, Barcelona, Sept. 2013, pp. 138–143.
- C. Rösmann, F. Hoffmann and T. Bertram: Planning of Multiple Robot Trajectories in Distinctive Topologies, Proc. IEEE European Conference on Mobile Robots, UK, Lincoln, Sept. 2015.
- C. Rösmann, F. Hoffmann and T. Bertram: Kinodynamic Trajectory Optimization and Control for Car-Like Robots, IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), Vancouver, BC, Canada, Sept. 2017.

<a href="https://www.buymeacoffee.com/croesmann" target="_blank"><img src="https://cdn.buymeacoffee.com/buttons/lato-black.png" alt="Buy Me A Coffee" height="31px" width="132px" ></a>

## Videos

The left of the following videos presents features of the package and shows examples from simulation and real robot situations.
Some spoken explanations are included in the audio track of the video. 
The right one demonstrates features introduced in version 0.2 (supporting car-like robots and costmap conversion). Please watch the left one first.

<a href="http://www.youtube.com/watch?feature=player_embedded&v=e1Bw6JOgHME" target="_blank"><img src="http://img.youtube.com/vi/e1Bw6JOgHME/0.jpg" 
alt="teb_local_planner - An Optimal Trajectory Planner for Mobile Robots" width="240" height="180" border="10" /></a>
<a href="http://www.youtube.com/watch?feature=player_embedded&v=o5wnRCzdUMo" target="_blank"><img src="http://img.youtube.com/vi/o5wnRCzdUMo/0.jpg" 
alt="teb_local_planner - Car-like Robots and Costmap Conversion" width="240" height="180" border="10" /></a>

## License

The *teb_local_planner* package is licensed under the BSD license.
It depends on other ROS packages, which are listed in the package.xml. They are also BSD licensed.

Some third-party dependencies are included that are licensed under different terms:
 - *Eigen*, MPL2 license, http://eigen.tuxfamily.org
 - *libg2o* / *g2o* itself is licensed under BSD, but the enabled *csparse_extension* is licensed under LGPL3+, 
   https://github.com/RainerKuemmerle/g2o. [*CSparse*](http://www.cise.ufl.edu/research/sparse/CSparse/) is included as part of the *SuiteSparse* collection, http://www.suitesparse.com. 
 - *Boost*, Boost Software License, http://www.boost.org

All packages included are distributed in the hope that they will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the licenses for more details.

## Requirements

Install dependencies (listed in the *package.xml* and *CMakeLists.txt* file) using *rosdep*:

    rosdep install teb_local_planner



Command continuity retains the admitted request and route frame as its identity.
The route frame may differ from the local costmap frame: the existing native
plan transform must succeed before any command is computed, and output twists
remain in the robot base frame. A changed route identity clears continuity
history. Enabled station-envelope geometry still requires the local costmap
frame; this scope identity does not transform or relabel station coordinates.

The selected-command obstacle check sweeps the actual filled padded convex
footprint along the selected planar twist. It includes angular corner motion
between samples and conservatively tests the enlarged polygons against complete
occupied or unknown costmap cells under one map lock. Map bounds, existing
padding and the declared nominal horizon remain mandatory. Nonfinite, degenerate,
nonconvex or unordered footprints fail closed. This nominal geometric check does
not establish actuator tracking, stopping distance or dynamic-obstacle safety.

If the observed map initially places the padded footprint against a known
obstacle, surge/yaw robots may use contact recovery. The optimizer's command is
projected onto a small deterministic set containing route-directed forward and
reverse surge, curved surge/yaw motion, and in-place turns; every candidate must
pass the same exact swept-hull check before use. Recovery is limited to the
connected obstacle component touched at the initial pose. Existing measured
free clearance may be used by a maneuver, but the raw hull may not cross the
observed boundary. If localization already places the raw hull inside a wall,
its initial support is the worst allowed value and may not deepen. New
protrusions, unrelated obstacles, unknown space, and worsening physical
penetration retain ordinary collision rejection. Corner boundaries retire
independently after the padded hull clears them. This handles apparent contact
from map or localization error without deleting obstacles or using simulator
truth.

Station mode penalizes distance from the requested station center throughout the trajectory, while retaining separate hard nominal envelope and command-arc checks. This centering objective does not certify physical tracking or localization error.

## Differential-drive trajectory initialization

Robots with zero lateral velocity and zero minimum turning radius initialize
reference paths with pivot and drive segments, preserving the start and goal
headings. The optimizer may smooth these segments. Pose-only and path inputs
share this initialization, including the requested final-velocity mode.

The differential-drive lateral-motion cost integrates squared lateral velocity
over each existing time interval, so subdividing the same timed path does not
reduce its penalty. Positive time intervals keep colocated rotation poses
regular. This remains a soft optimization constraint; it does not replace
collision checking or prove physical tracking accuracy.
