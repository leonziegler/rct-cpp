rct
===

Robotics Coordinate Transform (C++)

This library wraps the functionality of the tf2 library from ROS and
supports communication over the RSB middleware.


Dependencies
------------

Required dependencies:
- tf2 minimal installation\*
- boost
- Eigen3
- RSC (Robotic System Commons)

Optional dependencies:
- RSB
- roscpp

\* for a minimal installation of tf2 see section *TF2 Minimal Installation*

Installation
------------

	mkdir build && cd build
	cmake ..
	make
	make install

In order to disable ROS support use:

	cmake -DBUILD_ROS_SUPPORT=OFF ..

TF2 Minimal Installation
------------------------

	git clone TODO
