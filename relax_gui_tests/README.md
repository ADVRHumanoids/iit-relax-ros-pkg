#Relax new GUI stuff
1 - ssh -X embedded@192.168.12.77
2 - export uri on gedit (pilot)
2 - ecat_master
3 - xbot2-core --hw ec_imp
4 - mon launch mir_driver mir.launch
5 - cd src/gui_relax & mon launch launch_relax.launch
6 - (GUI WEB) http://cloud.tactilerobots.com:8080/

Settings/Server
    ROS URL: ws://192.168.12.77:9090
    ROS SIM URL: ws://cloud.tactilerobots.com:9090
    MESH SRV URL: ws://cloud.tactilerobots.com:8081
    Robot Description: /xbotcore/robot_description
    Joint State: /xbotcore/joint_state
