FROM ros:humble-ros-base

# environment variables
ENV DEBIAN_FRONTEND=noninteractive

# install dependencies
RUN apt update
RUN apt install -y \
    build-essential \
    python3 \
    git \
    xauth \
    wget \
    python3-rosdep

# init rosdep
RUN rm /etc/ros/rosdep/sources.list.d/20-default.list
RUN rosdep init
RUN rosdep update

# copy the ros package and build it
RUN mkdir -p /spac_ws/src/wonderful_package
WORKDIR /spac_ws/src/
#clone into existing package
RUN git clone https://github.com/FSLART/Spac2.git -b eufs_sim_devel wonderful_package 

WORKDIR /spac_ws
# install dependencies
RUN rosdep install --from-paths /spac_ws --ignore-src -r -y
# build
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    cd /spac_ws && \
    colcon build --parallel-workers 6 --symlink-install"

# launch the package
#CMD /bin/bash -c "source /opt/ros/humble/setup.bash && \
#    source install/setup.bash && \
#    ros2 launch spac2_0 drivemodel.launch.xml"

CMD ["/bin/bash"]   