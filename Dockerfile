FROM ros:humble

# Set the working directory
WORKDIR /root

# Copy the repository into the container
COPY . /root/dynablox

# Update the package list and install dependencies
RUN apt-get update && apt-get install -y \
    libeigen3-dev \
    libgoogle-glog-dev \
    libyaml-cpp-dev \
    libboost-system-dev \
    libboost-thread-dev \
    libboost-program-options-dev \
    libboost-test-dev \
    && rm -rf /var/lib/apt/lists/*

# Source the ROS environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Set the entrypoint to bash
ENTRYPOINT ["/bin/bash"]
