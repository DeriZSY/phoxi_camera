roslaunch phoxi_camera phoxi_camera.launch& 
sleep 2s
roslaunch phoxi_camera phoxi_camera_cal.launch num_of_iteration:="2" image_path:="/home/bionicdl/calibration_images/im2"

kill -15 %1
