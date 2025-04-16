# issues

used the dockerfile from the repo.

didn't work out of the box.

started working backward from the prerequisites they listed.

> 1. Prerequisites
> 1.1 Ubuntu and ROS
> Ubuntu 64-bit 16.04 or 18.04. ROS Kinetic or Melodic. ROS Installation
> 1.2. Ceres Solver
> 1.3. PCL

used kinetic to start. didn't work.

moved to melodic.

replaced all the git clone pcl and ceres repos and building from source w/ sudo apt-get installs of the public packages.

hit a issue w/ docker on mac m1 machines. failing hash checksums during apt-get commands. only solution was to turn off privacy & restrictions on mac settings and restart.
^ one of the worst bugs ever lol

default python on melodic is python2.

default python after python-pip install was < 3.6 and failed pip install evo command.

used deadsnakes apt repository to install specific python version.

dockerfile is finally building.

able to run dockerfile w/ rviz after some brutal debugging.

originally attempted to run w/ mesa drivers and software only rendering. failed getting this to work w/ rviz.

pivoted to using NVIDIA GPU. updated docker run command and was able to get everything working.

