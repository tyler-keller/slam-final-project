#!/bin/bash

while getopts ":r" flag;
do
    case "${flag}" in
        r) rviz=1
    esac
done

if [ -z "${rviz}" ]; then 
	rviz=0
fi

xhost +local:docker

docker run -dit --rm \
    --privileged \
    --net=host \
    --gpus all \
    -e DISPLAY=$DISPLAY \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e LIBGL_ALWAYS_INDIRECT=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -v "${PWD}/data":/data \
    aloam

container_id=$(docker run -dit --rm \
    --privileged \
    --net=host \
    --gpus all \
    -e DISPLAY=$DISPLAY \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e LIBGL_ALWAYS_INDIRECT=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -v "${PWD}/data":/data \
    aloam \
    tail -f /dev/null)

run_in_container() {
    docker exec "${container_id}" /bin/bash -c "source /opt/ros/melodic/setup.bash && source /root/catkin_ws/devel/setup.bash && $1"
}

# remapping 00.bag topic to expected aloam topic
run_in_container "rosrun topic_tools relay /kitti/velo/pointcloud /velodyne_points >/dev/null &"

# running aloam
run_in_container "roslaunch aloam_velodyne aloam_velodyne_HDL_64.launch rviz:=${rviz} >/dev/null &"
sleep 1

# recording trajectory to bag
run_in_container "rosbag record /aft_mapped_to_init -O /data/output.bag __name:=aloam_bag >/dev/null &"
run_in_container "rosbag play /data/00.bag"
run_in_container "rosnode kill /aloam_bag"

# d3p3ndency errors... weird
run_in_container "python3.7 -m pip install tkinter"

# restructuring .bag to .tum and running relevant evo commands
mma cv
evo_traj bag output.bag /aft_mapped_to_init --save_as_tum
evo_traj tum aft_mapped_to_init.tum sequence_00_GT.txt --ref=sequence_00_GT.txt --save_as_tum --plot_mode=xz --save_plot=traj_xz.pdf --no_warnings
evo_ape tum sequence_00_GT.txt aft_mapped_to_init.tum --align --correct_scale --save_results=ape.json --save_plot=ape.pdf --no_warnings
evo_rpe tum sequence_00_GT.txt aft_mapped_to_init.tum --align --correct_scale --save_results=rpe.json --save_plot=rpe.pdf --no_warnings

docker kill "${container_id}"
xhost -local:docker
