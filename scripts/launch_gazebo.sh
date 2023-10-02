FILE_DIR=$(realpath $(dirname $0))

export GAZEBO_MODEL_PATH=$GAZEBO_MODELS_PATH:$FILE_DIR/../assets/models
ros2 launch gazebo_ros gazebo.launch.py world:=$FILE_DIR/../assets/worlds/cafe.world \
                                        extra_gazebo_args:="--verbose"
                                        #gdb:=true