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

const double CashKarp54::c[6] = { 0.0, 1.0 / 5.0, 3.0 / 10.0, 3.0 / 5.0, 1.0, 7.0 / 8.0 };
const double CashKarp54::b5th[6] = { 37.0 / 378.0, 0.0, 250.0 / 621.0, 125.0 / 594.0, 0.0, 512.0 / 1771.0 };
const double CashKarp54::b4th[6] = { 2825.0 / 27648.0, 0.0, 18575.0 / 48384.0, 13525.0 / 55296.0, 277.0 / 14336.0, 1.0 / 4.0 };
const double CashKarp54::bDiff[6] = { b5th[0] - b4th[0], b5th[1] - b4th[1], b5th[2] - b4th[2], b5th[3] - b4th[3], b5th[4] - b4th[4], b5th[5] - b4th[5] };
const double CashKarp54::a[6][5] = {
	{},
	{ 1.0 / 5.0 },
	{ 3.0 / 40.0, 9.0 / 40.0 },
	{ 3.0 / 10.0, -9.0 / 10.0, 6.0 / 5.0 },
	{ -11.0 / 54.0, 5.0 / 2.0, -70.0 / 27.0, 35.0 / 27.0 },
	{ 1631.0 / 55296.0, 175.0 / 512.0, 575.0 / 13824.0, 44275.0 / 110592.0, 253.0 / 4096.0 } };

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

