# Novint Falcon ros nodes for multirotor offboard control

I described my work on this project on my blog:

* [Part 1](https://msadowski.github.io/1ppm/uav-haptic-control-pt1/)
* [Part 2](https://msadowski.github.io/1ppm/uav-haptic-control-pt2/)

## Getting started

### libnifalcon
To compile this project you will need [libnifalcon library](https://github.com/libnifalcon/libnifalcon) compiled and installed.

Via libnifalcon instructions:

```
sudo apt-get install libusb-1.0-0-dev cmake-curses-gui libboost-all-dev
mkdir build
cd build
cmake -G "Unix Makefiles" ..
make
make install
```

Check if the library was installed successfully using the command:
`pkg-config --libs libnifalcon`

Try launch *findfalcons* by running in the terminal: `sudo findfalcons`.
Be wary of [this issue](https://github.com/libnifalcon/libnifalcon/issues/45).

If your falcon is recognized then copy [40-novint-falcon-udev.rules](https://github.com/libnifalcon/libnifalcon/blob/master/linux/40-novint-falcon-udev.rules) to */etc/udev/rules.d/* directory.

### Px4 SITL

* Follow the [Px4 docs](https://dev.px4.io/simulation-sitl.html) to run jmavsim.

### Falcon node

You will need to create a ROS workspace, and then clone this repository. To launch the ROS nodes use the launch file:
`roslaunch offb_node falcon.launch`

If you don't have experience with ROS then do the tutorials at [http://wiki.ros.org/ROS/Tutorials](http://wiki.ros.org/ROS/Tutorials). 

## Contributing

Feel free to fork this repo and do pull requests. I'm not likely to actively support this project however if you run into issues then please open an issue on github.
