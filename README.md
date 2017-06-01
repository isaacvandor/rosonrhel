# ROS on RHEL7

### Steps To Get ROS Running Properly on Rhel 7 (Note: All steps apply to RHEL7 and have not been verified with RHEL6 or other Fedora-based distros)

# Prerequisites
*Install Rhel 7
(For this tutorial, RHEL7 installed via virtualbox)

1. Install EPEL (Extra Packages) and Free/Nonfree packages for RHEL7:
  [Tutorial Link](https://www.cyberciti.biz/faq/installing-rhel-epel-repo-on-centos-redhat-7-x/)
  [Official instructions for EPEL download](http://fedoraproject.org/wiki/EPEL/FAQ#howtouse)
  
 ` sudo yum localinstall --nogpgcheck https://download1.rpmfusion.org/free/el/rpmfusion-free-release-7.noarch.rpm https://download1.rpmfusion.org/nonfree/el/rpmfusion-nonfree-release-7.noarch.rpm `

2. Install Python (If not already installed)
3. Install Python-docutils
4. Install Pip
  * Use ` sudo yum pip install ____ ` to install all required dependencies

# Installing ROS
* Follow ROS source install [Instructions](http://wiki.ros.org/Installation/Source)
    
# Catkin Build System
## Install Prerequisites
(Note: Must be installed individually)
* Cmake
* Python (Should already be installed)
  * catkin_pkg
  * empy
  * nose
* gtest
* GNU C++ 
