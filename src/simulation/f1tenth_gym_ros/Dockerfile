# MIT License

# Copyright (c) 2020 Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

FROM ros:humble

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        git \
        nano \
        vim \
        python3-pip \
        python3-venv \
        python3-dev \
        libeigen3-dev \
        tmux \
        ros-humble-rviz2 && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /sim_ws
RUN mkdir -p /sim_ws/src/f1tenth_gym_ros
COPY . /sim_ws/src/f1tenth_gym_ros

RUN python3 -m venv --system-site-packages /sim_ws/.venv && \
    source /sim_ws/.venv/bin/activate && \
    pip install -U pip && \
    pip install -e /sim_ws/src/f1tenth_gym_ros/f1tenth_gym

ENV VIRTUAL_ENV=/sim_ws/.venv
ENV PATH="$VIRTUAL_ENV/bin:$PATH"

RUN source /opt/ros/humble/setup.bash && \
    apt-get update && \
    if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then \
        rosdep init; \
    fi && \
    rosdep update && \
    rosdep install -i --from-paths /sim_ws/src --rosdistro humble -y && \
    colcon build --symlink-install

ENTRYPOINT ["/bin/bash"]
