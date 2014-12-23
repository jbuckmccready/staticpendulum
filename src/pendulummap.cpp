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

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <chrono>
#include "pendulummap.hpp"
#include "lodepng/lodepng.h"

PendulumMap::PendulumMap() :
  m_resolution{0.1},
  m_xStart{-10.0},
  m_yStart{-10.0},
  m_xEnd{10.0},
  m_yEnd{10.0},
  m_timeStart{0.0},
  m_startingStepSize{0.001},
  m_positionTolerance{0.5},
  m_midPositionTolerance{0.1},
  m_timeTolerance{5.0},
  m_progress{0}
{
  m_midConvergeColor = RGBAColor{ 0, 0, 0, 255 };
  m_outOfBoundsColor = RGBAColor{ 255, 255, 255, 255 };
  m_attractorColorTable.emplace_back(RGBAColor{255, 140, 0, 255});
  m_attractorColorTable.emplace_back(RGBAColor{30, 144, 255, 255});
  m_attractorColorTable.emplace_back(RGBAColor{178, 34, 34, 255});
  initializeMapData();
}

void PendulumMap::saveMapPNGImage(std::string fileName)
{
  std::vector<unsigned char> image;
  image.resize(m_mapData.size() * 4);
  std::size_t bufferIndex = 0;
  for (auto iter = m_mapData.begin(); iter != m_mapData.end(); ++iter) {
    if(iter->convergePosition == -2) { // check if it's out of bounds
      image[bufferIndex] = m_outOfBoundsColor.r;
      image[bufferIndex+1] = m_outOfBoundsColor.g;
      image[bufferIndex+2] = m_outOfBoundsColor.b;
      image[bufferIndex+3] = m_outOfBoundsColor.a;
    } else if(iter->convergePosition == -1) { // check if it converged to the middle
      image[bufferIndex] = m_midConvergeColor.r;
      image[bufferIndex+1] = m_midConvergeColor.g;
      image[bufferIndex+2] = m_midConvergeColor.b;
      image[bufferIndex+3] = m_midConvergeColor.a;
    } else { // check which attractor it converged to
      for(int i = 0, size = static_cast<int>(m_attractorColorTable.size()); i < size; ++i) {
        if(iter->convergePosition == i) {
          image[bufferIndex] = m_attractorColorTable[i].r;
          image[bufferIndex+1] = m_attractorColorTable[i].g;
          image[bufferIndex+2] = m_attractorColorTable[i].b;
          image[bufferIndex+3] = m_attractorColorTable[i].a;
          break;
        }
      }
    }
    bufferIndex += 4;
  }

  // check if fileName has png extension and if it doesn't then append it
  if(fileName.rfind(".png") != fileName.size() - 4)
    fileName.append(".png");

  unsigned error = lodepng::encode(fileName, image, m_cols, m_rows);
  if(error)
    std::cout << "Image encoder error " << error << ": " << lodepng_error_text(error) << std::endl;
}

void PendulumMap::saveMapTSV(std::string fileName)
{
  // check if fileName has tsv extension and if it doesn't then append it
  if(fileName.rfind(".tsv") != fileName.size() - 4)
    fileName.append(".tsv");

  std::ofstream file(fileName);
  if(!file.is_open()) {
    std::cout << "Failed to save file." << std::endl;
    return;
  }

  for (unsigned int i = 0; i < m_rows; ++i) {
    for(unsigned int j = 0; j < m_cols; ++j) {
      file << m_mapData[i*m_cols + j].convergePosition << "\t";
    }
    file << "\n";
  }
  file.close();

}

//! Set the x and y start and end positions and the resolution.
void PendulumMap::setMapArea(double xStart, double xEnd, double yStart, double yEnd, double resolution)
{
  m_xStart = xStart;
  m_xEnd = xEnd;
  m_yStart = yStart;
  m_yEnd = yEnd;
  m_resolution = resolution;
  initializeMapData();
}

//! Set the converge tolerance for stopping the integration.
void PendulumMap::setConvergeTolerance(double positionTolerance, double midPositionTolerance, double timeTolerance)
{
  m_positionTolerance = positionTolerance;
  m_midPositionTolerance = midPositionTolerance;
  m_timeTolerance = timeTolerance;
}

//! Set the initial step size for the integrator.
void PendulumMap::setStartingStepSize(double stepSize)
{
  m_startingStepSize = stepSize;
}

void PendulumMap::addAttractorColor(unsigned char r, unsigned char g, unsigned char b, unsigned char a)
{
  m_attractorColorTable.emplace_back(RGBAColor{r, g, b, a});
}

void PendulumMap::setAttractorColor(unsigned char r, unsigned char g, unsigned char b, unsigned char a, int index)
{
  m_attractorColorTable[index].r = r;
  m_attractorColorTable[index].g = g;
  m_attractorColorTable[index].b = b;
  m_attractorColorTable[index].a = a;
}

void PendulumMap::setMidConvergeColor(unsigned char r, unsigned char g, unsigned char b, unsigned char a)
{
  m_midConvergeColor.r = r;
  m_midConvergeColor.g = g;
  m_midConvergeColor.b = b;
  m_midConvergeColor.a = a;
}

void PendulumMap::setOutOfBoundsColor(unsigned char r, unsigned char g, unsigned char b, unsigned char a)
{
  m_outOfBoundsColor.r = r;
  m_outOfBoundsColor.g = g;
  m_outOfBoundsColor.b = b;
  m_outOfBoundsColor.a = a;
}

void PendulumMap::clearAttractorColors()
{
  m_attractorColorTable.clear();
}

void PendulumMap::launchProgressBar()
{
  bool done = false;
  const std::chrono::milliseconds waitTime(10);
  unsigned long long loopCount = 0;
  const unsigned int pointCount = m_rows*m_cols;
  float percentProgress = 0.0;
  std::string progressBar;
  progressBar.reserve(25);
  std::string spaceFill(26, ' ');
  std::ostringstream percentStrStream;

  std::chrono::time_point<std::chrono::system_clock> startTime, endTime;
  startTime = std::chrono::system_clock::now();
  std::cout << "Integrating the map...\n";
  while(!done) {
    // update the progress bar every 32 * waitTime milliseconds
    if(loopCount % 32 == 0) {
      percentProgress = static_cast<float>(m_progress)/static_cast<float>(pointCount) * 100.0f;
      std::size_t barCount = std::lround(percentProgress/4.0f);

      // update the hashtag progress bar
      while(progressBar.length() < barCount) {
        progressBar.push_back('#');
        spaceFill.pop_back();
      }

      // create percent progress string and output to system shell
      percentStrStream << std::fixed << std::setprecision(2) << percentProgress;
      std::cout << "\r" + progressBar + spaceFill + percentStrStream.str() + "%" << std::flush;

      // clear string stream for next percent value
      percentStrStream.str("");
      percentStrStream.clear();
    }

    std::this_thread::sleep_for(waitTime);

    // check every loop iteration to see if we're done integrating
    if(m_progress == pointCount) {
      done = true;
      endTime = std::chrono::system_clock::now();
      std::chrono::duration<double> elapsedSeconds = endTime - startTime;
      if(progressBar.length() != 25)
        while(progressBar.length() < 25) {
          progressBar.push_back('#');
          spaceFill.pop_back();
        }

      std::cout << "\r" + progressBar + spaceFill + "100.00" + "%";
      std::cout << "\nDone! Integration took " << elapsedSeconds.count() << " seconds.\n";
    }
    ++loopCount;
  }
}

void PendulumMap::initializeMapData()
{
  m_mapData.clear();
  m_cols = std::abs(std::lround((m_xEnd-m_xStart)/m_resolution))+1;
  m_rows = std::abs(std::lround((m_yEnd-m_yStart)/m_resolution))+1;
  m_mapData.resize(m_rows*m_cols);
  int xdim_factor = std::lround(m_xStart/m_resolution); // create int multipliers to fill data to avoid floating math rounding error
  int ydim_factor = std::lround(m_yStart/m_resolution);
  std::size_t index;
  for (std::size_t i = 0; i < m_rows; ++i) {
    for (std::size_t j = 0; j < m_cols; ++j) {
      index = i*m_cols + j; // data is row major oriented
      m_mapData[index].xPosition = static_cast<double>(xdim_factor) * m_resolution;
      m_mapData[index].yPosition = static_cast<double>(ydim_factor) * -m_resolution; // y factor is negative such that the first element corresponds to the upper left of the map
      m_mapData[index].xVelocity = 0.0;
      m_mapData[index].yVelocity = 0.0;
      ++xdim_factor;
    }

    xdim_factor = std::lround(m_xStart/m_resolution); // reset x value for next row
    ++ydim_factor;
  }
}
