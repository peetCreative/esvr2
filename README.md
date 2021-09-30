# esvr2

This is a component in the research project _EndoMersion_ for advanced laparoscopic image viewing in VR and camera work using head gestures. 
It is a prototypical VR-Application to serve these two main purposes:
1. Display stereo-laparoscopic images in VR-Environment for improved spatial perception.
2. Enable manipulating image section using different driven head-gesture methodes. 

It's meant to be configurable in certain sense:
1. Position, Distance/ Scale of the projection plane is adjustable
2. Distortion of the images

This project is part of my master thesis project.
Therefore, it is likely to be discontinued at the end of the year.
For more progress please have a look at publications of [TCO group of NCT Dresden](https://www.nct-dresden.de/forschung/departments-and-groups/department-for-translational-surgical-oncology.html).

More [documentation](https://peetcreative.github.io/esvr2/)

This VR-Application is designed as a library and can be easly be started from another application e.g. a [ROS-Node](https://github.com/peetCreative/esvr2_ros).
However it can also be run as a standalone.

## Install

1. Check SteamVR, OpenVR, yaml-cpp is installed
2. manually clone and compile [ogre-next](https://github.com/OGRECave/ogre-next)
   install it to sth else than `/opt/ogre`
3. create build directory
4. run `cmake` with parameter
   `-DOGRE_HOME=/path/to/ogre/install`
   * optional also define `-DCMAKE_INSTALL_PREFIX=/path/to/esvr2/install`
5. `make`
6. `make install`