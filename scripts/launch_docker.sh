FILE_DIR=$(realpath $(dirname $0))
ROS_WS=$FILE_DIR/../../../../ros2_ws

xhost +local:root

docker run -it --name middleware_test_cont --rm --privileged -v /tmp/.X11-unix:/tmp/.X11-unix \
                                                             -e XAUTHORITY=$XAUTHORITY \
                                                             -e DISPLAY=$DISPLAY \
                                                             -v $ROS_WS:/root/ros2_ws/ \
                                                             middleware_test:humble 

xhost -local:root


