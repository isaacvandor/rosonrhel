# ROS on RHEL7

Steps To Get ROS Running Properly on Rhel 7 (Note: All steps apply to RHEL7 and have not been verified with RHEL6 or other Fedora-based distros)


# Prerequisites
* Install Rhel 7
[For this tutorial, RHEL7 installed via virtualbox](https://www.virtualbox.org/wiki/Downloads)
(Note: Also install Virtualbox Extension Pack)


1. Install EPEL (Extra Packages) and Free/Nonfree packages for RHEL7:
  * [Tutorial Link](https://www.cyberciti.biz/faq/installing-rhel-epel-repo-on-centos-redhat-7-x/)
  * [Official instructions for EPEL download](http://fedoraproject.org/wiki/EPEL/FAQ#howtouse)
  
 ` sudo yum localinstall --nogpgcheck https://download1.rpmfusion.org/free/el/rpmfusion-free-release-7.noarch.rpm https://download1.rpmfusion.org/nonfree/el/rpmfusion-nonfree-release-7.noarch.rpm `

2. [Install Python (If not already installed)](https://packaging.python.org/install_requirements_linux/)
3. [Install Python-docutils](https://www.rpmfind.net/linux/rpm2html/search.php?query=python-docutils)
* ` subscription-manager register --auto-attach `
* ` subscription-manager attach --auto `
* ` subscription-manager repos --enable rhel-7-server-optional-rpms --enable rhel-7-server-extras-rpms `

4. [Install Pip](https://www.liquidweb.com/kb/how-to-install-pip-on-centos-7/)
  * Use ` sudo yum pip install ____ ` to install all required dependencies
  * Note: Most packages needed for installation can be installed through [PyPi](https://pypi.python.org/pypi)

# Installing ROS
* Follow ROS source install [Instructions](http://wiki.ros.org/Installation/Source)

` Use ` sudo yum pip install ____ ` to install all required dependencies `

Under the resolving dependencies section, run ` rosdep install --from-paths src --ignore-src --rosdistro <ENTER YOUR DISTRO HERE> --os=fedora:21 `
    
# Catkin Build System
## Install Prerequisites
(Note: Must be installed individually and most can be installed using ` sudo yum pip install ____ ` to install all required dependencies)
* [Cmake](https://cmake.org/download/)
 1. Download tar.gz file
 Navigate to location of download and run:
 * ` tar xvzf PACKAGENAME.tar.gz `
 * ` ./configure `
 * ` make `
 * ` sudo make install `
* Python (Should already be installed)
  * [catkin_pkg](http://wiki.ros.org/catkin_pkg)
  * [empy](http://www.alcyone.com/pyos/empy/)
  * [nose](https://nose.readthedocs.io/en/latest/)
* [gtest](http://wiki.ros.org/gtest)
* [GNU C++](https://gcc.gnu.org/) 
## Install Catkin Build System from Source
[Follow Section 2.2 Install from Source](http://wiki.ros.org/catkin)


## Other Attempts:

### ROS Using Docker Containerization
Failed because Docker install on RHEL7 is unsupported at the moment

* [Docker for RHEL Installation Instructions](docs.docker.com/engine/installation/linux/rhel/)
* [Running ROS using Docker](https://store.docker.com/images/ros)

# Success!

## Install Docker on RHEL7
* [Docker Rhel7 Install Instructions](http://docs.master.dockerproject.org/engine/installation/linux/rhel/)

## Links
* [Docker Store ROS Page](https://store.docker.com/images/ros)
* [OSRF Docker Images](https://github.com/osrf/docker_images)
* [ROS Documentation on Docker](http://wiki.ros.org/docker)


## Steps
1. Pull latest ROS image from Docker
   ` sudo yum install docker-engine `
2. Start Docker Engine
   ` sudo service docker start `
3. Verify Docker Engine is running properly
   ` sudo docker run hello-world `
4. Run ` sudo yum update `
5. Pull latest ROS Image
   ` docker pull ros `
6. Run ROS container
   ` docker run -it ros `
7. Verify ROS is working properly by running roscore
   (Roscore should now be running. To continue, open a new terminal window. To quite at any point, type ctrl-c and exit the terminal to kill the ROS container and Docker engine.)
8. Check for any active docker containers
   ` docker ps -l `
9. Run container in new terminal window
   ` docker exec -it YOUR ID NAME HERE bash `
10. Source new ROS container
   ` source /ros_entrypoint.sh `
11. Verify ROS is running properly
   ` rostopic list `
   
ROS should now be running in a Docker container with a master node. See link below to learn to develop new nodes etc.

### ROS Using Snap Packages
* Installed dnf `sudo yum install dnf `
* [snapd is unsupported currently on RHEL7](https://snapcraft.io/docs/core/install)
