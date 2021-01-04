# esvr2
jet annother stereo rendering project to towards openvr

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