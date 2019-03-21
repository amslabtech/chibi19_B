map_serverを使うときは
sudo apt-get install libsdl1.2-dev
sudo apt install libsdl-image1.2-dev
この２つをインストールして
~/ros_catkin_ws/srcで
git clone https://github.com/ros-planning/navigation.git
を実行し catkin_make する必要がある。

~/ros_catkin_ws/src/chibi19_b/mapdata$ rosrun map_server map_server DMapData.yaml

で実行可能。
