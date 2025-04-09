FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# Install base dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    libeigen3-dev \
    python3 \
    python3-pip \
    pkg-config \
    && apt-get clean

# Install Python packages for visualization
RUN pip3 install pandas matplotlib plotly numpy

RUN apt-get update && apt-get install -y libyaml-cpp-dev

# === Build OSQP from source ===
# WORKDIR /opt
# # RUN git clone https://github.com/osqp/osqp.git && \
# #     cd osqp && mkdir build && cd build && \
# #     cmake .. && make -j4 && make install

# === Download Webots Headless CLI (R2023b) ===
# NOTE: Headless mode doesn't support GUI or rendering, so we only use Webots CLI tools for structure.
# === Install Webots (latest available for Ubuntu 22.04) ===
# RUN wget https://cyberbotics.com/wwi/R2023b/webots_2023b-rev1_amd64.deb && \
#     apt install -y ./webots_2023b-rev1_amd64.deb && \
#     rm webots_2023b-rev1_amd64.deb


    # Set working directory to project root
WORKDIR /usr/src/mpc_bicycle_tracking

# Default command
CMD ["bash"]
