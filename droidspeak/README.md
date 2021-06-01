droidspeak
==========

Package Summary
---------------

Make your robot speak like R2-D2

- Maintainer status: maintained
- Maintainer: Noël Martignoni (noel.martignoni AT easymov DOT fr)
- Author: Noël Martignoni
- License: BSD
- Source: git git@gitlab.com/easymov/droidspeak.git (branch: master)

Requirements
------------

- python >=2.6
- pygame

Overview
--------

Use this node to make your robot speak like R2-D2

Quick start
-----------

Clone this repository into the source directory of a valid catkin workspace
then build it and don't forget to source the setup file in the devel directory of your catkin workspace.

`roslaunch droidspeak droidspeak.launch`

You can write some text into the *speak* and listen to the sweet voice of R2-D2.

`rostopic pub /droidspeak std_msgs/String hello `

Node
----

### droidspeak ###

#### Subscribed Topics ####

_~speak_ (std_msgs/String)
 > String that will be read in droidspeak

### Launch file ###

`roslaunch droidspeak droidspeak.launch`