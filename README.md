# Abstract
A damped electrostatic pendulum is modeled using a system of differential equations and solved using numerical integration methods. A two dimensional map of initial conditions is integrated to construct a colored fractal image formed by the discrete points of convergence. Multiple programmatic solutions were implemented using Mathematica, Python and C++ to compare and improve computational performance. A decrease of two orders of magnitude in computation time was obtained by writing an adaptive step Runge Kutta integration method in C++. With significantly improved computation times high resolution images are constructed and stitched together to form videos that demonstrate how the system progressively changes with varying physical parameters: gravity, air drag, attractor strength, etc. The optimized code enables further statistical analysis, and may be easily modified for other systems of differential equations, initial condition maps, and integration methods.

# Compiling
CMakeLists.txt included. External libraries used: lodepng (https://github.com/lvandeve/lodepng) and jsoncpp (https://github.com/open-source-parsers/jsoncpp). Source for those libraries is included directly in this repository for simplicity but could be better maintained using submodules.

No external libraries are required outside of this repository, but C++11/14 features and libraries are used (greatest performance found using GCC 4.9 using -ffastmath, -O3, etc.). I have only tested on two machines: an AMD Phenom II 720BE with unlocked 4th core and overclocked to 3.3Ghz with Windows 7, and an Intel i7-3630QM Ubuntu laptop. I found GCC to be the fastest, next Clang, and finally MSVC. GCC is very effective at optimizing the tight loops with floating point operations, and I was unable to speed up the integration by writing inline sse intrinsic instructions. However MSVC is significantly slower and may be sped up by writing some sse intrinsic instructions.

# Usage
The program takes in a json map configuration file and png image file name. An example configuration file is provided (map.json).
Example use:
$staticpendulum.exe map.json mapImage.png

# Additional Information
Check the Presentation.pdf for example map images and general overview of the project.
