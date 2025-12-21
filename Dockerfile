FROM ros:jazzy

# Avoid interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-venv \
    python3-scipy \
    git \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages for KISS-SLAM pipeline
# Note: We use the Python-based kiss-slam package which doesn't require ROS packages
RUN pip3 install --break-system-packages \
    kiss-slam \
    rosbags \
    numpy \
    open3d \
    scipy \
    typer \
    pydantic \
    laspy[laszip]

# Create workspace
WORKDIR /ws

# Copy scripts directory
COPY scripts/ /ws/scripts/
RUN chmod +x /ws/scripts/*.sh

# Copy the run script to a convenient location
RUN cp /ws/scripts/run_pipeline.sh /run_pipeline.sh && \
    chmod +x /run_pipeline.sh

# Set entrypoint
ENTRYPOINT ["/bin/bash", "-c", "source /opt/ros/jazzy/setup.bash && exec \"$@\"", "--"]
CMD ["bash"]
