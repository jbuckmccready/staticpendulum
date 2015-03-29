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

#ifndef PENDULUMMAP_HPP
#define PENDULUMMAP_HPP
#include <vector>
#include <cmath>
#include <future>
#include <atomic>
#include <algorithm>
#include "pendulumsystem.hpp"
#include "threadpool.hpp"

//! Class used to store and integrate maps of initial conditions for the
// pendulum system.
class PendulumMap {
public:
  //! Struct used to store point information.
  struct Point {
    double xPosition;
    double yPosition;
    double xVelocity;
    double yVelocity;
    int convergePosition = -2; // -2 reserved for points that are out of bounds,
                               // -1 for points that converge to the middle
    double convergeTime = 0.0;
    unsigned int stepCount = 0;
  };

  //! Struct used to store RGBA values for each attractor
  struct RGBAColor {
    unsigned char r;
    unsigned char g;
    unsigned char b;
    unsigned char a;
  };

  PendulumMap();
  template <typename Integrator>
  void parallelIntegrateMap(const Integrator &theIntegrator,
                            const PendulumSystem &theSystem,
                            unsigned int threadCount);

  void saveMapPNGImage(std::string fileName);
  void saveMapTSV(std::string fileName);

  void setMapArea(double xStart, double xEnd, double yStart, double yEnd,
                  double resolution);
  void setConvergeTolerance(double positionTolerance,
                            double midPositionTolerance, double timeTolerance);
  void setStartingStepSize(double stepSize);

  void addAttractorColor(unsigned char r, unsigned char g, unsigned char b,
                         unsigned char a);
  void setAttractorColor(unsigned char r, unsigned char g, unsigned char b,
                         unsigned char a, int index);
  void setMidConvergeColor(unsigned char r, unsigned char g, unsigned char b,
                           unsigned char a);
  void setOutOfBoundsColor(unsigned char r, unsigned char g, unsigned char b,
                           unsigned char a);
  void clearAttractorColors();

private:
  std::vector<Point> m_mapData; // data of the map
  // rows and columns of the map
  std::size_t m_rows;
  std::size_t m_cols;

  // rgba color information for saving images
  std::vector<RGBAColor> m_attractorColorTable;
  RGBAColor m_midConvergeColor;
  RGBAColor m_outOfBoundsColor;

  double m_resolution; // resolution of the map
  // start and end points for the map
  double m_xStart;
  double m_yStart;
  double m_xEnd;
  double m_yEnd;
  double m_timeStart;        // starting integration time
  double m_startingStepSize; // starting integration step size
  double
      m_positionTolerance; // position tolerance for checking magnet convergence
  double m_midPositionTolerance; // position tolerance for checking mid/gravity
                                 // convergence
  double m_timeTolerance;        // time tolerance for checking convergence
  std::atomic<unsigned int>
      m_progress; // progress counter value for progress bar

  template <typename Integrator>
  void integratePoint(Integrator &theIntegrator,
                      const PendulumSystem &theSystem, Point &thePoint);

  template <typename Integrator, typename Iterator>
  void integratePointRange(const Integrator &theIntegrator,
                           const PendulumSystem &theSystem, Iterator start,
                           Iterator end);
  void launchProgressBar();
  void initializeMapData();
};

//! Parellel integrate the map inside this object given some integrator, system,
// and number of threads
template <typename Integrator>
void PendulumMap::parallelIntegrateMap(const Integrator &theIntegrator,
                                       const PendulumSystem &theSystem,
                                       unsigned int threadCount) {
  // break the map up into groups of points rather than pushing every point as a
  // single task into the thread pool queue
  const int group = std::max(static_cast<int>(m_mapData.size()) / 5000, 1);
  auto it = m_mapData.begin();
  auto last = m_mapData.end();
  ThreadPool pool(threadCount);
  for (; it < last - group; it += group) {
    // lambda wrapper used to pass the function, handles type deduction
    pool.addTask([=, &theIntegrator, &theSystem]() {
      integratePointRange(theIntegrator, theSystem, it,
                          std::min(it + group, last));
    });
  }
  pool.addTask([=, &theIntegrator, &theSystem]() {
    integratePointRange(theIntegrator, theSystem, it, last);
  });                  // task out the remainder set of points
  launchProgressBar(); // goes until finished

  //  // multithreaded integration of the map
  //  const unsigned int group = (m_mapData.size())/threadCount;
  //  std::vector<std::thread> threads;
  //  threads.reserve(threadCount);
  //  auto it = m_mapData.begin();
  //  auto last = m_mapData.end();
  //  // function pointer to resolve compiler parsing UPDATE: Lambda cleaner and
  //  no pointer cleanup needed (compiled on gcc 4.8.1)
  //  //    void (pendulum_map<integrator_type>::*fxn_ptr)(const
  //  integrator_type&, const pendulum_system&, map_iter, map_iter) const =
  //  &pendulum_map<integrator_type>::integrate_map;
  //  for (; it < last-group; it += group) {
  //    //        threads.push_back(std::thread(fxn_ptr, this,
  //    std::ref(the_integrator), std::ref(the_system), it, std::min(it+group,
  //    last)));
  //    threads.push_back(std::thread([=,&theIntegrator,&theSystem]()
  //    {integratePointRange(theIntegrator, theSystem, it, std::min(it+group,
  //    last));}));
  //  }
  //  integratePointRange(theIntegrator, theSystem, it, last); // left over
  //  chunks while we wait for other threads
  //  launchProgressBar();
  //  std::for_each(threads.begin(), threads.end(), [](std::thread&
  //  x){x.join();}); // wait for all threads to finish integrations
}

//! Integrate a single point of type point_type
template <typename Integrator>
inline void PendulumMap::integratePoint(Integrator &theIntegrator,
                                        const PendulumSystem &theSystem,
                                        PendulumMap::Point &thePoint) {
  // increment progress atomic for progress bar
  ++m_progress;

  // check if the point is within the pendulum length boundary
  if (std::sqrt(std::pow(thePoint.xPosition, 2) +
                std::pow(thePoint.yPosition, 2)) > (theSystem.LENGTH - 1e-10))
    return;

  // check if the point is (0,0) as it is undefined by our pendulum system
  if (std::abs(thePoint.xPosition) < 1e-10 &&
      std::abs(thePoint.yPosition) < 1e-10)
    return;

  // integration good to go, create local state for integration to keep start
  // state
  PendulumSystem::StateType current_state = {
      {thePoint.xPosition, thePoint.yPosition, thePoint.xVelocity,
       thePoint.yVelocity}};
  auto isNearAttractor =
      [](double attX, double attY, double currX, double currY, double tol) {
        return ((attX - tol < currX) && (currX < attX + tol) &&
                (attY - tol < currY) && (currY < attY + tol));
      };

  auto isNearMiddle = [](double currX, double currY, double tol) {
    return ((-tol < currX) && (currX < tol) && (-tol < currY) && (currY < tol));
  };

  double currTime = m_timeStart;
  double stepSize = m_startingStepSize;
  int trialCount = 0;
  bool converged = false;
  double initialTimeFound = 0.0;
  bool nearAttractor = false; // start not near any attractor
  int currentAttractor = -2;
  while (!converged && trialCount < 1000000) {
    thePoint.stepCount +=
        theIntegrator(theSystem, current_state, currTime, stepSize);
    ++trialCount;

    // check if pendulum head near an attractor and if it's been near for long
    // enough time to consider converged
    for (int i = 0,
             attractorCount = static_cast<int>(theSystem.attractorList.size());
         i < attractorCount; ++i) {
      if (isNearAttractor(theSystem.attractorList[i].xPosition,
                          theSystem.attractorList[i].yPosition,
                          current_state[0], current_state[1],
                          m_positionTolerance)) {
        nearAttractor = true;
        if (currentAttractor == i) {
          if (currTime - initialTimeFound > m_timeTolerance) {
            thePoint.convergeTime = currTime;
            thePoint.convergePosition = i;
            converged = true;
          }
        } else {
          currentAttractor = i;
          initialTimeFound = currTime;
        }
        break; // escape for-loop after finding that we're near an attractor
      }
    }
    if (!nearAttractor) {
      // check if pendulum head near middle
      if (isNearMiddle(current_state[0], current_state[1],
                       m_midPositionTolerance)) {
        if (currentAttractor == -1) {
          if (currTime - initialTimeFound > m_timeTolerance) {
            thePoint.convergeTime = currTime;
            thePoint.convergePosition = -1;
            converged = true;
          }
        } else {
          currentAttractor = -1;
          initialTimeFound = currTime;
        }
      }
    }
    // ELSE: not near middle or any attractor, just go to next step
    nearAttractor = false; // set flag for next step
  }
}

template <typename Integrator, typename Iterator>
void PendulumMap::integratePointRange(const Integrator &theIntegrator,
                                      const PendulumSystem &theSystem,
                                      Iterator start, Iterator end) {
  for (Iterator it = start; it != end; ++it) {
    integratePoint(theIntegrator, theSystem, *it);
  }
}
#endif // PENDULUMMAP_HPP
