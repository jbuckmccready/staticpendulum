/* ===========================================================================
 * The MIT License (MIT)
 *
 * Copyright (c) 2014 Jedidiah Buck McCready <jbuckmccready@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 * ===========================================================================*/

#include <vector>

#include "integrators/cashkarp54.hpp"
#include "pendulummap.hpp"
#include "pendulumsystem.hpp"
#include "readconfig.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <regex>


int main(int argc, char *argv[])
{
  if(argc != 2 && argc != 3) {
    std::cout << "To integrate a map you must supply a single map .cfg file and output image file name." << std::endl;
    std::cout << "Example use of the program: \"$staticpendulum mymapconfig.cfg mymapimage.png\"" << std::endl;
    return 0;
  }

  std::string configFileName = argv[1];
  std::string imageFileName = argv[2];

  std::ifstream file(configFileName);
  std::stringstream stringBuffer;
  stringBuffer << file.rdbuf();
  std::string configString = stringBuffer.str();

  // system parameters
  double distance = doubleParameterFromConfig(configString, "distance", 0.05);
  double mass = doubleParameterFromConfig(configString, "mass", 1.0);
  double gravity = doubleParameterFromConfig(configString, "gravity", 9.8);
  double drag = doubleParameterFromConfig(configString, "drag", 0.2);
  double length = doubleParameterFromConfig(configString, "length", 10.0);
  std::vector<PendulumSystem::Attractor> attractors;
  std::vector<PendulumSystem::Attractor> defaultAttractors;
  defaultAttractors.emplace_back(PendulumSystem::Attractor{-0.5, std::sqrt(3.0)/2.0, 1.0});
  defaultAttractors.emplace_back(PendulumSystem::Attractor{-0.5, -std::sqrt(3.0)/2.0, 1.0});
  defaultAttractors.emplace_back(PendulumSystem::Attractor{1.0, 0.0, 1.0});
  fillAttractorsFromConfig(configString, "attractors", defaultAttractors, attractors);

  // map parameters
  double x_start = doubleParameterFromConfig(configString, "x_start", -10.0);
  double y_start = doubleParameterFromConfig(configString, "y_start", -10.0);
  double x_end = doubleParameterFromConfig(configString, "x_end", 10.0);
  double y_end = doubleParameterFromConfig(configString, "y_end", 10.0);
  double resolution = doubleParameterFromConfig(configString, "resolution", 0.05);
  double attractor_position_tolerance = doubleParameterFromConfig(configString, "attractor_position_tolerance", 0.5);
  double mid_position_tolerance = doubleParameterFromConfig(configString, "mid_position_tolerance", 0.1);
  double time_tolerance = doubleParameterFromConfig(configString, "time_tolerance", 5.0);
  double starting_step_size = doubleParameterFromConfig(configString, "starting_step_size", 0.001);

  std::vector<PendulumMap::RGBAColor> attractor_colors;
  std::vector<PendulumMap::RGBAColor> defaultColors;
  defaultColors.emplace_back(PendulumMap::RGBAColor{255, 140, 0, 255});
  defaultColors.emplace_back(PendulumMap::RGBAColor{30, 144, 255, 255});
  defaultColors.emplace_back(PendulumMap::RGBAColor{178, 34, 34, 255});
  fillColorsFromConfig(configString, "attractor_colors", defaultColors, attractor_colors);

  std::vector<PendulumMap::RGBAColor> middle_converge_color;
  std::vector<PendulumMap::RGBAColor> defaultMidConvergeColor;
  defaultMidConvergeColor.emplace_back(PendulumMap::RGBAColor{0, 0, 0, 255});
  fillColorsFromConfig(configString, "middle_converge_color", defaultMidConvergeColor, middle_converge_color);

  std::vector<PendulumMap::RGBAColor> out_of_bounds_color;
  std::vector<PendulumMap::RGBAColor> defaultOutOfBoundsColor;
  defaultOutOfBoundsColor.emplace_back(PendulumMap::RGBAColor{255, 255, 255, 255});
  fillColorsFromConfig(configString, "out_of_bounds_color", defaultOutOfBoundsColor, out_of_bounds_color);

  // integration parameters
  double relative_tolerance = doubleParameterFromConfig(configString, "relative_tolerance", 1e-6);
  double absolute_tolerance = doubleParameterFromConfig(configString, "absolute_tolerance", 1e-6);
  double maximum_step_size = doubleParameterFromConfig(configString, "maximum_step_size", 0.1);
  int thread_count = std::lround(doubleParameterFromConfig(configString, "thread_count", 8));

  PendulumSystem mySystem;
  PendulumMap myMap;
  CashKarp54 myIntegrator;

  // set system
  mySystem.DISTANCE = distance;
  mySystem.MASS = mass;
  mySystem.GRAVITY = gravity;
  mySystem.DRAG = drag;
  mySystem.LENGTH = length;
  mySystem.clearAttractors();
  for (const auto &attractor : attractors) {
    mySystem.addAttractor(attractor.xPosition, attractor.yPosition, attractor.forceCoeff);
  }

  // set map
  myMap.setMapArea(x_start, x_end, y_start, y_end, resolution);
  myMap.setConvergeTolerance(attractor_position_tolerance, mid_position_tolerance, time_tolerance);
  myMap.setStartingStepSize(starting_step_size);
  myMap.clearAttractorColors();
  for(const auto &color : attractor_colors) {
    myMap.addAttractorColor(color.r, color.g, color.b, color.a);
  }
  myMap.setMidConvergeColor(middle_converge_color[0].r, middle_converge_color[0].g, middle_converge_color[0].b, middle_converge_color[0].a);
  myMap.setOutOfBoundsColor(out_of_bounds_color[0].r, out_of_bounds_color[0].g, out_of_bounds_color[0].b, out_of_bounds_color[0].a);

  // set integrator
  myIntegrator.setTolerance(relative_tolerance, absolute_tolerance);
  myIntegrator.setMaxStepSize(maximum_step_size);

  // integrate map and save
  myMap.parallelIntegrateMap(myIntegrator, mySystem, thread_count);
  myMap.saveMapPNGImage(imageFileName);
  return 0;
}
