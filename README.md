# quad_anim
This is a small ROS2 package that animates a quadcopter. Code structure is relatively simple right now. The `scripts\genGeomVects.py` file uses SymPy to come up with the equations for different position vectors, which are converted into C code.

There is an `animWindowNode` which contains subscribers for the body pose, motor angles and time-stamp. The new state of the quadcopter is rendered when the time-stamp is received.

A simple 3D line based model is plotted using Matplot++ library. Some properties of the model, like the size of the quadcopter or the diameter of the propellers, are hardcoded right now. These will eventually be extracted using ROS2 services, from a Simulation.

Currently, Matplot++ is not the best solution for this since the plot flickers on the update. The eventual goal is to render a quadcopter with 3D models. So, some possible candidate that can replace Matplot++ are OGRE, Easy3D and morphologica libraries.