# SPAC2.0

No not the Finance Derivate, Simple PID Ackerman Control

PID and Pure Pursuit implemented

#### Note:

The launch file has a parameter file hardcoded this is a major issue with this package.

## Setup ["docker"]
Assuming you have docker engine running, simply build it

```bash
docker build -t spac .
```

To run, run it with host network in order to see the topics on your local machine (you also need the ipc=host)

```bash
docker run -it --net=host --ipc=host spac
```

## Setup ["native"]

Assuming you have ros humble installed in your local machine, copy or clone this package into the /src of your workspace, after that you will need to install dependencies. Go to your workspace directory and run the following

```bash
rosdep install --from-paths src --ignore-src -y -r
```

Build It

```bash
colcon build --parallel-workers 6 --symlink-install
```
After that you need to source it

```bash
source install/setup.bash
```
And you are ready to launch it

```bash
ros2 launch spac2_0 drivemodel.launch.xml
```