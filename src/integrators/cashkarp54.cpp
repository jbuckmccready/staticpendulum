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

#include "cashkarp54.hpp"
/*!
 * \brief Set error tolerances for the integrator.
 * \param relativeTolerance The relative error tolerance, controls the steps relative to the size of the step taken.
 * \param absoluteTolerance The absolute error tolerance, controls the steps independent of the size of the step taken.
 *
 * \details Absolute tolerance is independent of the step taken while relative tolerance is weighted by the step taken.
 * The mathematical formula used is the same as the one used by the boost odeint library:
 * http://www.boost.org/doc/libs/1_53_0/libs/numeric/odeint/doc/html/boost_numeric_odeint/odeint_in_detail/steppers.html#boost_numeric_odeint.odeint_in_detail.steppers.controlled_steppers
 */

void CashKarp54::setTolerance(double relativeTolerance, double absoluteTolerance)
{
  m_relativeTolerance = relativeTolerance;
  m_absoluteTolerance = absoluteTolerance;
}

//! Set the maximum step size the integrator is allowed to take.
void CashKarp54::setMaxStepSize(double maxStepSize)
{
  m_maxStepSize = maxStepSize;
}

