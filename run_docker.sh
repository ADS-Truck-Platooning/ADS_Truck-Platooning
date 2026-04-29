#!/bin/bash

docker run --gpus all -it --rm \
  --net=host \
  --ipc=host \
  --shm-size=2g \
  --privileged \
  -e DISPLAY=$DISPLAY \
  -e NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute \
  -e __GL_SYNC_TO_VBLANK=0 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v ~/carla:/carla \
  -v ~/ADS_Truck-Platooning:/ADS_Truck-Platooning \
  -e CARLA_ROOT=/carla \
  -e LD_LIBRARY_PATH=/carla/PythonAPI/carla/dependencies/lib:$LD_LIBRARY_PATH \
  -e PYTHONPATH=/carla/PythonAPI/carla:$PYTHONPATH \
  junspring/ads-truck-platooning bash