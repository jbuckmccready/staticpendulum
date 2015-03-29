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
#include "jsoncpp/json.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <regex>

int main(int argc, char *argv[]) {
  if (argc != 2 && argc != 3) {
    std::cout << "To integrate a map you must supply a single parameters "
                 "configuration json file "
                 "and output image file name." << std::endl;
    std::cout << "Example use of the program: \"$staticpendulum "
                 "parameters.json mymapimage.png\"" << std::endl;
    return 1;
  }

  std::string configFileName = argv[1];
  std::string imageFileName = argv[2];

  std::ifstream file(configFileName);
  std::stringstream stringBuffer;
  stringBuffer << file.rdbuf();
  std::string configString = stringBuffer.str();
  file.close();
  Json::Value root;
  Json::Reader reader;
  bool parsingSuccessful = reader.parse(configString, root);
  if (!parsingSuccessful) {
    std::cout << "Failed to parse configuration\n"
              << reader.getFormattedErrorMessages();
    return 1;
  }

  std::cout << "\n******************************\n";
  std::cout << "SYSTEM PARAMETERS\n";
  std::cout << "******************************\n";
  double pendulumHeight = root["system"]["distance"].asDouble();
  double pendulumMass = root["system"]["mass"].asDouble();
  double gravity = root["system"]["gravity"].asDouble();
  double drag = root["system"]["drag"].asDouble();
  double pendulumLength = root["system"]["length"].asDouble();
  std::cout << "Distance: " << pendulumHeight << "\nMass: " << pendulumMass
            << "\nGravity: " << gravity << "\nDrag: " << drag
            << "\nLength: " << pendulumLength << "\n";
  std::vector<PendulumSystem::Attractor> attractors;

  const Json::Value configAttractors = root["system"]["attractors"];

  for (unsigned int i = 0; i < configAttractors.size(); ++i) {
    std::cout << "Attractor[" << i << "]: ("
              << configAttractors[i]["X"].asDouble() << ", "
              << configAttractors[i]["Y"].asDouble() << ", "
              << configAttractors[i]["Z"].asDouble() << ")\n";
    attractors.emplace_back(
        PendulumSystem::Attractor{configAttractors[i]["X"].asDouble(),
                                  configAttractors[i]["Y"].asDouble(),
                                  configAttractors[i]["Z"].asDouble()});
  }

  std::cout << "\n******************************\n";
  std::cout << "MAP PARAMETERS\n";
  std::cout << "******************************\n";
  double x_start = root["map"]["xStart"].asDouble();
  double y_start = root["map"]["yStart"].asDouble();
  double x_end = root["map"]["xEnd"].asDouble();
  double y_end = root["map"]["yEnd"].asDouble();
  double resolution = root["map"]["resolution"].asDouble();
  double attractor_position_tolerance =
      root["map"]["attractorPositionTolerance"].asDouble();
  double mid_position_tolerance =
      root["map"]["midPositionTolerance"].asDouble();
  double time_tolerance = root["map"]["timeTolerance"].asDouble();
  double starting_step_size = root["map"]["startingStepSize"].asDouble();
  std::cout << "xStart: " << x_start << "\nyStart: " << y_start
            << "\nxEnd: " << x_end << "\nyEnd: " << y_end
            << "\nresolution: " << resolution
            << "\nattractorPositionTolerance: " << attractor_position_tolerance
            << "\nmidPositionTolerance: " << mid_position_tolerance
            << "\ntimeTolerance: " << time_tolerance
            << "\nstartingStepSize: " << starting_step_size << "\n";

  std::vector<PendulumMap::RGBAColor> attractor_colors;
  const Json::Value configColors = root["map"]["attractorColors"];

  for (unsigned int i = 0; i < configColors.size(); ++i) {
    int r = configColors[i]["r"].asInt();
    int g = configColors[i]["g"].asInt();
    int b = configColors[i]["b"].asInt();
    int a = configColors[i]["a"].asInt();
    std::cout << "AttractorColor[" << i << "]: (" << r << ", " << g << ", " << b
              << ", " << a << ")\n";
    attractor_colors.emplace_back(PendulumMap::RGBAColor{
        static_cast<unsigned char>(r), static_cast<unsigned char>(g),
        static_cast<unsigned char>(b), static_cast<unsigned char>(a)});
  }

  int midR = root["map"]["middleConvergeColor"]["r"].asInt();
  int midG = root["map"]["middleConvergeColor"]["g"].asInt();
  int midB = root["map"]["middleConvergeColor"]["b"].asInt();
  int midA = root["map"]["middleConvergeColor"]["a"].asInt();

  std::cout << "middleConvergeColor: (" << midR << ", " << midG << ", " << midB
            << ", " << midA << ")\n";

  PendulumMap::RGBAColor middle_converge_color{
      static_cast<unsigned char>(midR), static_cast<unsigned char>(midG),
      static_cast<unsigned char>(midB), static_cast<unsigned char>(midA)};

  int boundsR = root["map"]["outOfBoundsColor"]["r"].asInt();
  int boundsG = root["map"]["outOfBoundsColor"]["g"].asInt();
  int boundsB = root["map"]["outOfBoundsColor"]["b"].asInt();
  int boundsA = root["map"]["outOfBoundsColor"]["a"].asInt();

  std::cout << "outOfBoundsColor: (" << boundsR << ", " << boundsG << ", "
            << boundsB << ", " << boundsA << ")\n";

  PendulumMap::RGBAColor out_of_bounds_color{
      static_cast<unsigned char>(boundsR), static_cast<unsigned char>(boundsG),
      static_cast<unsigned char>(boundsB), static_cast<unsigned char>(boundsA)};

  std::cout << "\n******************************\n";
  std::cout << "INTEGRATION PARAMETERS\n";
  std::cout << "******************************\n";
  double relative_tolerance =
      root["integration"]["relativeTolerance"].asDouble();
  double absolute_tolerance =
      root["integration"]["absoluteTolerance"].asDouble();
  double maximum_step_size = root["integration"]["maximumStepSize"].asDouble();
  int thread_count = root["integration"]["threadCount"].asInt();
  std::cout << "relativeTolerance: " << relative_tolerance
            << "\nabsoluteTolerance: " << absolute_tolerance
            << "\nmaximumStepSize: " << maximum_step_size
            << "\nthreadCount: " << thread_count << "\n";

  PendulumSystem pendulumSystem;
  PendulumMap pendulumMap;
  CashKarp54 pendulumIntegrator;

  // set system
  pendulumSystem.DISTANCE = pendulumHeight;
  pendulumSystem.MASS = pendulumMass;
  pendulumSystem.GRAVITY = gravity;
  pendulumSystem.DRAG = drag;
  pendulumSystem.LENGTH = pendulumLength;
  pendulumSystem.clearAttractors();
  for (const auto &attractor : attractors) {
    pendulumSystem.addAttractor(attractor.xPosition, attractor.yPosition,
                                attractor.forceCoeff);
  }

  // set map
  pendulumMap.setMapArea(x_start, x_end, y_start, y_end, resolution);
  pendulumMap.setConvergeTolerance(attractor_position_tolerance,
                                   mid_position_tolerance, time_tolerance);
  pendulumMap.setStartingStepSize(starting_step_size);
  pendulumMap.clearAttractorColors();
  for (const auto &color : attractor_colors) {
    pendulumMap.addAttractorColor(color.r, color.g, color.b, color.a);
  }
  pendulumMap.setMidConvergeColor(
      middle_converge_color.r, middle_converge_color.g, middle_converge_color.b,
      middle_converge_color.a);
  pendulumMap.setOutOfBoundsColor(out_of_bounds_color.r, out_of_bounds_color.g,
                                  out_of_bounds_color.b, out_of_bounds_color.a);

  // set integrator
  pendulumIntegrator.setTolerance(relative_tolerance, absolute_tolerance);
  pendulumIntegrator.setMaxStepSize(maximum_step_size);

  // integrate map and save
  pendulumMap.parallelIntegrateMap(pendulumIntegrator, pendulumSystem,
                                   thread_count);
  pendulumMap.saveMapPNGImage(imageFileName);
  return 0;
}
