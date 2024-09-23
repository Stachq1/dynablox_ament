FROM ros:humble

# Set the working directory
WORKDIR /root

# Copy the repository into the container
COPY . /root/dynablox

# Update the package list and install dependencies
RUN apt-get update && apt-get install -y \
    libgoogle-glog-dev \
    libyaml-cpp-dev \
    libboost-system-dev \
    libboost-thread-dev \
    protobuf-compiler \
    libpcl-dev \
    vim \
    && rm -rf /var/lib/apt/lists/*

# Remove apt installed Eigen 3.4.0 and install 3.3.7 instead
RUN rm -rf /usr/include/eigen3 && mv /root/dynablox/eigen3 /usr/include/

# Source the ROS environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Set the entrypoint to bash
ENTRYPOINT ["/bin/bash"]
