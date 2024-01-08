echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source /f1tenth/dev_ws/install/local_setup.bash" >> ~/.bashrc
source ~/.bashrc

# install ros dependencies (not ideal having this here but whatever)
rosdep update
sudo apt-get update --fix-missing && rosdep install -i --from-path dev_ws/src --rosdistro humble -y
