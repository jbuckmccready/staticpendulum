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

#include "pendulumsystem.hpp"

//! Default constructor sets default pendulum properties, and k = 1.0 for three attractors positioned at: \f$(-0.5, \sqrt{3}/2)\f$, \f$(-0.5, -\sqrt{3}/2)\f$, and \f$(1, 0)\f$.
PendulumSystem::PendulumSystem() :
  DISTANCE{0.05},
  MASS{1.0},
  GRAVITY{9.8},
  DRAG{0.2},
  LENGTH{10.0}
{
  attractorList.emplace_back(Attractor{-0.5, std::sqrt(3.0)/2.0, 1.0});
  attractorList.emplace_back(Attractor{-0.5, -std::sqrt(3.0)/2.0, 1.0});
  attractorList.emplace_back(Attractor{1.0, 0.0, 1.0});
}

//! Add an attractor at position (xPosition, yPosition) with attractive force coefficient forceCoeff.
void PendulumSystem::addAttractor(double xPosition, double yPosition, double forceCoeff = 1.0)
{
  attractorList.emplace_back(Attractor{xPosition, yPosition, forceCoeff});
}

//! Set new position and attraction strength for already existing attractor at an index.
void PendulumSystem::setAttractor(int index, double xPosition, double yPosition, double forceCoeff)
{
  attractorList[index].xPosition = xPosition;
  attractorList[index].yPosition = yPosition;
  attractorList[index].forceCoeff = forceCoeff;
}

//! Set all the attractor strengths to the same value.
void PendulumSystem::setAllAttractorStrengths(double forceCoeff)
{
  for (auto &attractor : attractorList) {
    attractor.forceCoeff = forceCoeff;
  }
}

//! Clear all attractors.
void PendulumSystem::clearAttractors()
{
  attractorList.clear();
}
