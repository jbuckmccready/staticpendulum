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

#ifndef PENDULUMSYSTEM_HPP
#define PENDULUMSYSTEM_HPP
#include <cmath>
#include <array>
#include <vector>

//! Pendulum function object that returns the derivative of the current state.

/*! The pendulum system is described by the following system of differential equations:
 *
 * \f$\vec{F}_{g}=-mg\frac{\sqrt{1-\frac{x^2+y^2}{L^2}}}{L}(x\hat{i}+y\hat{j})\f$ (force due to gravity)
 *
 * \f$\vec{F}_{a_n(x_n,y_n)}=\frac{-k[(x-x_n)\hat{i}+(y-y_n)\hat{j}]}{\left[(x-x_n)^2+(y-y_n)^2+\left(d+L-\sqrt{L^2-\left(x^2+y^2\right)}\right)\right]^{3/2}}\f$ (force due to attractor)
 *
 * \f$\vec{F}_{d}=-b(v_{x}\hat{i}+v_{y}\hat{j})\f$ (force due to dampening)
 *
 * \f$a_x=\frac{d^2x}{dt^2}=\left(F_{gx} + \sum\limits_{n=1}^{n=N}\vec{F}_{a_{nx}} + F_{dx}\right) \times \frac{1}{m}\f$ (acceleration in x direction)
 *
 * \f$a_y=\frac{d^2y}{dt^2}=\left(F_{gy} + \sum\limits_{n=1}^{n=N}\vec{F}_{a_{ny}} + F_{dy}\right) \times \frac{1}{m}\f$ (acceleration in y direction)
 *
 * Where m is the mass of the pendulum head, L is the length of the pendulum, g is the acceleration due to gravity, b is the dampening coefficient,
 * and d is the distance between the end of the pendulum at rest and the base plate of attractors.
 *
 * The system parameters are modified as public member variables while the attractors are modified by calling the respective member functions.
 *
 * The function is called using an overloaded () operator and returns through a reference parameter the derivative of the state passed in.
 */


struct PendulumSystem {
  typedef std::array< double , 4 > StateType; // container type for the state

  //! Container for an attractor, stores the position as an x-y coordinate, and an attractive force coefficient.
  struct Attractor {
    double xPosition; /*!< x coordinate position. */
    double yPosition; /*!< y coordinate position. */
    double forceCoeff; /*!< Attractive force coefficient where \f$F_{attractor} = \frac{-k}{x^2+y^2}\f$ */
  };

  double DISTANCE; /*!< Distance between the pendulum head at rest and the base plate. */
  double MASS; /*!< Mass of the head of the pendulum. */
  double GRAVITY; /*!< Acceleration due to gravity. */
  double DRAG; /*!< Linear drag coefficient. */
  double LENGTH; /*!< Length of the pendulum. */
  std::vector<Attractor> attractorList; /*!< List of attractors for the system. */

  PendulumSystem();
  void operator() (const StateType &x, StateType &dxdt, const double /* t */) const;
  void addAttractor(double xPosition, double yPosition, double forceCoeff);
  void setAttractor(int index, double xPosition, double yPosition, double forceCoeff);
  void setAllAttractorStrengths(double forceCoeff);
  void clearAttractors();
};


//! Function call that returns the derivative of the current state.
inline void PendulumSystem::operator() (const StateType &x /*!< Current state input; index 0 is x position, index 1 is y position, index 2 is x velocity, and index 3 is y velocity. */,
                                        StateType &dxdt /*!< Derivative of the state, value modified by reference; follows the same indexing as the input state. */,
                                        const double /*t*/ /*!< Note: system has no time dependence. Parameter here to fit signature for integration.*/) const
{
  // see latex equation or readme for more readable math, this is coded to minimize repeated calculations
  const double xSquared = x[0]*x[0];
  const double ySquared = x[1]*x[1];
  const double lengthSquared = LENGTH*LENGTH;
  const double normSquared = xSquared + ySquared;
  const double sqrtTerm = std::sqrt(1.0-normSquared/lengthSquared);

  const double gravityValue = -MASS*GRAVITY/LENGTH * sqrtTerm;

  double xAttractionForce = 0.0;
  double yAttractionForce = 0.0;

  const double value1 = DISTANCE+LENGTH*(1.0 - sqrtTerm);
  const double value2 = value1*value1;

  // sum up all the attractor forces
  for (auto attractor : attractorList) {
    double value3;
    double value4;
    double value5;
    double value6;
    double value7;
    value3 = x[0]-attractor.xPosition;
    value4 = x[1]-attractor.yPosition;
    value5 = value3*value3;
    value6 = value4*value4;
    value7 = -attractor.forceCoeff/std::pow(value5+value6+value2,1.5);

    xAttractionForce += value3*value7;
    yAttractionForce += value4*value7;
  }

  dxdt[0] = x[2];
  dxdt[1] = x[3];
  dxdt[2] = (x[0]*gravityValue - DRAG*x[2] + xAttractionForce) / MASS;
  dxdt[3] = (x[1]*gravityValue - DRAG*x[3] + yAttractionForce) / MASS;
}
#endif // PENDULUM_SYSTEM_H
