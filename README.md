# Abstract
A damped electrostatic pendulum is modeled using a system of differential equations and solved using numerical integration methods. A two dimensional map of initial conditions is integrated to construct a colored fractal image formed by the discrete points of convergence. Multiple programmatic solutions were implemented using Mathematica, Python and C++ to compare and improve computational performance. A decrease of two orders of magnitude in computation time was obtained by writing an adaptive step Runge Kutta integration method in C++. With significantly improved computation times high resolution images are constructed and stitched together to form videos that demonstrate how the system progressively changes with varying physical parameters: gravity, air drag, attractor strength, etc. The optimized code enables further statistical analysis, and may be easily modified for other systems of differential equations, initial condition maps, and integration methods.

# Compiling
Recommend using Qt Creator to open the staticpendulummapper.pro project file and compile for your platform using Qt Creator's interface. Alternatively one can use make tools and the Makefile provided in the build folder, or simply compile all the objects from the src, src/lodepng and src/integrators folders and link. No external libraries are required but C++11 features and libraries are used (greatest performance found using GCC 4.9).

# Usage
The program takes in a map configuration file and png image file name. An example configuration file is provided in the build folder.
Example use:
$staticpendulummapper standardmap.cfg standardmap.png

# Additional Information
Check the Presentation.pdf for example map images and general overview of the project.
