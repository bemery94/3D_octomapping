# Laser Scanner

## Overview

<INSERT_PACKAGE_OVERVIEW>

The <INSERT_PACKAGE_NAME> package has been tested under [ROS] Indigo and Ubuntu 14.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

**Authors: **

**Contact:** 

**Affiliation: Centre for Autonomous Systems (CAS), University of Technology, Sydney (UTS)**

***
## Installation

### Dependencies

 - [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
 -
 -
 -
 -

### Building

In order to install, clone the latest version from this repository into your catkin workspace and compile using:

    cd catkin_workspace/src
    git clone https://codeine.research.uts.edu.au/spir/ros.git
    cd ../
    catkin_make

Note. This will clone the entire workspace.

***
## Launch Files

### <INSERT_LAUNCH_FILE_NAME>

<INSERT_LAUNCH_FILE_DESCRIPTION>

#### Node diagram

![<INSERT_LAUNCH_FILE_NAME>](https://codeine.research.uts.edu.au/spir/ros/raw/master/indigo/<INSERT_PACKAGE_NAME>/doc/images/<INSERT_IMAGE_NAME>.jpg)

***
## Nodes

***

### [<INSERT_NODE_NAME>](<LINK_TO_DOXY_FILE>)

<INSERT_NODE_DESCRIPTION> 

#### Subscribed Topics

* ```<INSERT_TOPIC_NAME>``` ([<INSERT_MESSAGE_TYPE>])

    <INSERT_TOPIC_DESCRIPTION>

#### Published Topics

* ```<INSERT_TOPIC_NAME>``` ([<INSERT_MESSAGE_TYPE>])

    <INSERT_TOPIC_DESCRIPTION>

#### Services

* ```<INSERT_SERVICE_NAME>``` ([<INSERT_SERVICE_TYPE>])

    <INSERT_SERVICE_DESCRIPTION>


#### Parameters

* ```<INSERT_PARAM_NAME``` (<INSERT_PARAM_TYPE, default: "<INSERT_DEFAULT_VALUE_IF_APPLICABLE>")
    
    <INSERT_PARAM_DESCRIPTION>

***

### <INSERT_NEXT_NODE>

***
## Bugs & Feature Requests

<INSERT_BUGS_OR_FEATURE_REQUESTS_OR_DELETE_AND_LEAVE_LINE_BELOW>
There are no known bugs. Please report bugs and request features by emailing the author.

[ROS]: http://www.ros.org
[INSERT_MESSAGE_TYPE]: http://docs.ros.org/api/<MESSAGE_PACKAGE_NAME>/html/msg/<MESSAGE>.html
