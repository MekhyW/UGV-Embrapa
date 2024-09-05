## Docker Image

You can pull the Docker image from [DockerHub](https://hub.docker.com/layers/dustynv/ros/humble-ros-base-l4t-r36.2.0/images/sha256-deb93c6463f98442081ad5f81ff7fc816cc4374a6a36de818f7647e93fe07def?context=explore).

### Usage

```bash
docker pull dustynv/ros:humble-ros-base-l4t-r36.2.0
docker run -it --rm --runtime nvidia --network host dustynv/ros:humble-ros-base-l4t-r36.2.0
```