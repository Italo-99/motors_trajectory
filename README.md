# ARS Control Lab Internship: Robotiq85 Gripper

This repository contains some useful ROS1 files to handle the grippers (such as the Robotiq85 Gripper), carried out by the internship project of Italo Almirante within the Ars Control Laboratory, with the collaboration of the professor Cristian Secchi and the researcher Post-doctoral Andrea Pupa.

This is just a custom package for the practical easy use of the gripper, the proprietary package must be installed as well to import Robotiq85 Gripper urdf files, by using the following line:
git clone https://github.com/a-price/robotiq_arg85_description.git

## User guide

This package contains two classes: gripper and motor_mover. The former is child of the latter, so "motor_mover" can be used by external codes to handle everykind of motors.
