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

#include "readconfig.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <regex>


double doubleParameterFromConfig(const std::string &configString, const std::string &parameterName, double defaultValue)
{
  std::string::size_type startIndex;

  // read in and set the parameter, use defaultValue if not found
  if( (startIndex = configString.find(parameterName)) != std::string::npos) {

    // set iterators for regex search to go from the position of the parameter to the end of the line
    std::string::const_iterator searchStartIter = configString.begin() + startIndex;
    std::string::const_iterator searchEndIter = configString.begin() + configString.find_first_of("\n", startIndex);

    std::regex floatRegex("[-+]?[0-9]*\\.?[0-9]+([eE][-+]?[0-9]+)?");
    std::smatch match;

    if(std::regex_search(searchStartIter, searchEndIter, match, floatRegex))
      return std::stod(match[0].str());
  }

  // else could not find distance parameter
  std::cout << "Could not find value for " << parameterName << " parameter in cfg file." << std::endl;
  std::cout << "Using default value " << defaultValue << "." << std::endl;
  return defaultValue;
}

void fillAttractorsFromConfig(const std::string &fileString, const std::string &parameterName,
                             const std::vector<PendulumSystem::Attractor> &defaultAttractors, std::vector<PendulumSystem::Attractor> &attractors)
{
  std::string::size_type startIndex;
  if( (startIndex = fileString.find(parameterName)) != std::string::npos) {

    // set iterators for regex search to go from the position of the parameter to the end of the line
    std::string::const_iterator searchStartIter = fileString.begin() + startIndex;
    std::string::const_iterator searchEndIter = fileString.begin() + fileString.find_first_of("\n", startIndex);

    // regex to extract 3 float values in coordinate layout (x, y, f)
    std::regex attractorRegex("\\(\\s*([-+]?[0-9]*\\.?[0-9]+([eE][-+]?[0-9]+)?)\\s*,"
                              "\\s*([-+]?[0-9]*\\.?[0-9]+([eE][-+]?[0-9]+)?)\\s*,"
                              "\\s*([-+]?[0-9]*\\.?[0-9]+([eE][-+]?[0-9]+)?)\\s*\\)");

    auto attractorsBegin = std::sregex_iterator(searchStartIter, searchEndIter, attractorRegex);
    auto attractorsEnd = std::sregex_iterator();
    if(attractorsBegin != attractorsEnd) {
      for(auto i = attractorsBegin; i != attractorsEnd; ++i) {
        attractors.emplace_back(PendulumSystem::Attractor{std::stod((*i)[1].str()), std::stod((*i)[3].str()), std::stod((*i)[5].str())});
      }
      return;
    }
  }
  // else unable to find attractor values
  std::cout << "Could not find parameter values for system attractors, using default attractors:" << std::endl;
  for(const auto &attractor : defaultAttractors) {
    std::cout << "(" << attractor.xPosition << ", " << attractor.yPosition << ", " << attractor.forceCoeff << ")" << std::endl;
    attractors.emplace_back(attractor);
  }
}

void fillColorsFromConfig(const std::string &configString, const std::string &parameterName, const std::vector<PendulumMap::RGBAColor> &defaultColors,
                        std::vector<PendulumMap::RGBAColor> &colorTable)
{
  std::string::size_type startIndex;
  if( (startIndex = configString.find(parameterName)) != std::string::npos) {

    // set iterators for regex search to go from the position of the parameter to the end of the line
    std::string::const_iterator searchStartIter = configString.begin() + startIndex;
    std::string::const_iterator searchEndIter = configString.begin() + configString.find_first_of("\n", startIndex);

    // regex to extract 4 0-255 values in coordinate layout (r, g, b, a) for RGBA values
    std::regex colorRegex("\\(\\s*([0-9]|[1-9][0-9]|1[0-9][0-9]|2[0-4][0-9]|25[0-5])\\s*,"
                          "\\s*([0-9]|[1-9][0-9]|1[0-9][0-9]|2[0-4][0-9]|25[0-5])\\s*,"
                          "\\s*([0-9]|[1-9][0-9]|1[0-9][0-9]|2[0-4][0-9]|25[0-5])\\s*,"
                          "\\s*([0-9]|[1-9][0-9]|1[0-9][0-9]|2[0-4][0-9]|25[0-5])\\s*\\)");

    auto attractorsBegin = std::sregex_iterator(searchStartIter, searchEndIter, colorRegex);
    auto attractorsEnd = std::sregex_iterator();
    if(attractorsBegin != attractorsEnd) {
      for(auto i = attractorsBegin; i != attractorsEnd; ++i) {
        colorTable.emplace_back(PendulumMap::RGBAColor{static_cast<unsigned char>(std::stoi((*i)[1].str())),
                                                       static_cast<unsigned char>(std::stoi((*i)[2].str())),
                                                       static_cast<unsigned char>(std::stoi((*i)[3].str())),
                                                       static_cast<unsigned char>(std::stoi((*i)[4].str()))});
      }
      return;
    }
  }
  // else unable to find color values
  std::cout << "Could not find parameter values for attractor colors, using default RGBA colors:" << std::endl;
  for(const auto &color : defaultColors) {
    std::cout << "(" << static_cast<int>(color.r) << ", " << static_cast<int>(color.g) <<
                 ", " << static_cast<int>(color.b) << ", " << static_cast<int>(color.g) << ")" << std::endl;
    colorTable.emplace_back(color);
  }
}
