# Set all the environment variables
source ~/.bashrc
source /opt/ros/noetic/setup.sh
source ~/relax_mir_ws/install/setup.sh
source /opt/xbot/setup.sh
set_xbot2_config $HOME/relax_ws/src/iit-relax-ros-pkg/relax_config/relax_arm_basic.yaml
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:"$HOME/relax_ws/src/iit-relax-ros-pkg"

# Run the mesh server
$HOME/.nvm/versions/node/v16.13.2/bin/http-server "$HOME/relax_ws/src/iit-relax-ros-pkg" --cors -p 8081 &

# Run the "real" environment 
roslaunch relax_gazebo relax_world.launch gui:=false &
sleep 2; roslaunch mir_navigation amcl.launch initial_pose_x:=10.0 initial_pose_y:=10.0 &
sleep 2; roslaunch mir_navigation start_planner.launch map_file:=$(rospack find mir_gazebo)/maps/maze.yaml virtual_walls_map_file:=$(rospack find mir_gazebo)/maps/maze_virtual_walls.yaml &
#sleep 5; xbot2-core --hw dummy &
sleep 2; xbot2-core --simtime  --hw sim &
sleep 2; roslaunch $HOME/launch_relax.launch video_port:=8082

# Run the simulation environment
#export ROS_MASTER_URI=http://localhost:11411
#export GAZEBO_MASTER_URI=http://localhost:11355
#sleep 2; roslaunch -p 11411 relax_gazebo relax_world.launch gui:=false &
#sleep 2; roslaunch -p 11411 mir_navigation amcl.launch initial_pose_x:=10.0 initial_pose_y:=10.0 &
#sleep 2; roslaunch -p 11411 mir_navigation start_planner.launch map_file:=$(rospack find mir_gazebo)/maps/maze.yaml virtual_walls_map_file:=$(rospack find mir_gazebo)/maps/maze_virtual_walls.yaml &
#sleep 2; xbot2-core --hw dummy &
#sleep 2; xbot2-core --simtime  --hw sim &
#sleep 2; roslaunch -p 11411 ./launch_relax.launch port:=9091 video_port:=8083
