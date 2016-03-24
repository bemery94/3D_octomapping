# laser_conversions 

## Overview

The laser_conversions package uses the [laser_assembler] package to assemble the incoming laser_lsm
 scans from the vertical laser scanner. The call_laser_assembler_srv node is then run to get the
 assembled scans once every second, essentially producing one second blocks of assembled laser
 scans. These blocks of scans are passed into the slam_3D package to build the octomap. We
 assemble the scans as it reduces the overhead in the octomap package as it has to loop fewer
 times (e.g. it loops once per second for the block of scans rather than for each individual
 scan). This allows us to run the octomap at a finer resolution.

The laser_conversions package has been tested under [ROS] Indigo and Ubuntu 14.04. This is research
code, expect that it changes often and any fitness for a particular purpose is disclaimed.

**Authors:**

Brendan Emery

**Contact:** 

Brendan.Emery@student.uts.edu.au

**Affiliation: Centre for Autonomous Systems (CAS), University of Technology, Sydney (UTS)**

***
## Bugs & Feature Requests

There are no known bugs. Please report bugs and request features by emailing the author.

[ROS]: http://www.ros.org
[laser_assembler]: http://wiki.ros.org/laser_assembler