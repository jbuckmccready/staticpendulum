# Abstract
A damped electrostatic pendulum is modeled using a system of differential equations and solved using numerical integration methods. A two dimensional map of initial conditions is integrated to construct a colored fractal image formed by the discrete points of convergence. Multiple programmatic solutions were implemented using Mathematica, Python and C++ to compare and improve computational performance. A decrease of two orders of magnitude in computation time was obtained by writing an adaptive step Runge Kutta integration method in C++. With significantly improved computation times high resolution images are constructed and stitched together to form videos that demonstrate how the system progressively changes with varying physical parameters: gravity, air drag, attractor strength, etc. The optimized code enables further statistical analysis, and may be easily modified for other systems of differential equations, initial condition maps, and integration methods.

# Compiling
No external libraries are required but C++11 features and libraries are used (greatest performance found using GCC 4.9 using -ffastmath, -O3, etc.). I have only tested on two machines: an AMD Phenom II 720BE with unlocked 4th core and overclocked to 3.3Ghz with Windows 7, and an Intel i7-3630QM Ubuntu laptop. I found GCC to be the fastest, next Clang, and finally MSVC. GCC is effective at optimizing the tight loops for integration, and I was unable to speed up the integration by writing inline sse intrinsic instructions. However MSVC is significantly slower and may be sped up by writing some sse intrinsic instructions.

A Qt .pro project file is provided in the build folder for convenience.

# Usage
The program takes in a map configuration file and png image file name. An example configuration file is provided in the build folder.
Example use:
$staticpendulummapper standardmap.cfg standardmap.png

# Additional Information
Check the Presentation.pdf for example map images and general overview of the project.
