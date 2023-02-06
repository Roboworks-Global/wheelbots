FROM osrf/ros:galactic-desktop

RUN apt-get update

# Install tools
RUN apt-get install -y openssh-client vim python3-pip curl
            
# Install perception dependencies
RUN apt-get install -y libpcl-dev ros-galactic-pcl-msgs ros-galactic-pcl-ros
                
WORKDIR /app

ENTRYPOINT ["bash"]