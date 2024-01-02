# quad_anim
This is a small ROS2 package that animates a quadcopter. Code structure is relatively simple right now. The `scripts\genGeomVects.py` file uses SymPy to come up with the equations for different position vectors, which are converted into C code.

There is an `animWindowNode` which contains subscribers for the body pose, motor angles and time-stamp. The new state of the quadcopter is rendered when the time-stamp is received. This information is published by [quad_sim](https://github.com/vxj9800/quad_sim) node.

A simple 3D line based model is rendered using [RayLib](https://www.raylib.com/) library. So, RayLib is the only dependency of this package. Some properties of the model, like the size of the quadcopter or the diameter of the propellers, are hardcoded right now. These will eventually be extracted using ROS2 services, from the simulation.