gnome-terminal --tab --title="Launch" -- bash -c  "ros2 launch swerve_description simulation.launch.py;
                                                   echo Press any key to close;
                                                   read -n 1"



gnome-terminal --tab --title="Speed" -- bash -c "ros2 run swerve_control kinematic_node;
                                                    echo Press any key to close;
                                                    read -n 1"


gnome-terminal --tab --title="Joystick" -- bash -c "ros2 launch swerve_control joystick.launch.py;
                                                    echo Press any key to close;
                                                    read -n 1"