#############@blueberry#####################
ssh blueberry
vim .bashrc
roslaunch usb_cam usb_cam-test.launch
############################################

##############@wolf##########################
sudo chmod a+rw /dev/video0

rostopic info /usb_cam/image_row
rosmsg show sensor_msgs/Image
rviz→ add→ bytopic→ image(rqt_image_viewでも)
