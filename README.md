# gazebo_plugin_tutorials

These are some explorations to create a tricycle that can be controlled from ros.

# Building
To build the plugin create a build directory and make everything with

```
mkdir build
cd build
cmake ..
make
```

The plugins should now be build. To let Gazebo know where the plugins are, the directory has to be added to the plugin path

```
export GAZEBO_PLUGIN_PATH=`pwd`:$GAZEBO_PLUGIN_PATH
```

Gazebo can now be started. To see the stderr from the plugins, use `gazebo --verbose` to start Gazebo.

# Links
* [Gazebo Velodyne Plugin Tutorial](http://gazebosim.org/tutorials?cat=guided_i&tut=guided_i1)
* [Gazebo Build a Mobile Robot Tutorial](http://gazebosim.org/tutorials?tut=build_robot&cat=build_robot)
