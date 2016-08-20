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

#ifndef CASHKARP54_HPP
#define CASHKARP54_HPP
#include <algorithm>
#include <cmath>

/*!
 * \brief Cash and Karp embedded Runge Kutta order 5(4) adaptive step
 *integrator.
 *
 * See: J. R. Cash, A. H. Karp. “A Variable Order Runge-Kutta Method for Initial
 *Value Problems with
 * Rapidly Varying Right-Hand Sides.” ACM Transactions on Mathematical Software,
 *Vol. 16, No. 3, 1990.
 *
 * And Wikipedia: http://en.wikipedia.org/wiki/Cash-Karp_method,
 *http://en.wikipedia.org/wiki/List_of_Runge-Kutta_methods
 */

template <typename SystemType, typename StateType>
inline int CashKarp54(const SystemType &dxdt, StateType &x, double &t,
                      double &h, double relTol, double absTol,
                      double maxStepSize) {
  // Constants from Butcher tableau, see: http://en.wikipedia.org/wiki/Cash-Karp_method
  // and http://en.wikipedia.org/wiki/Runge-Kutta_methods
  const double c2 = 1.0 / 5.0;
  const double c3 = 3.0 / 10.0;
  const double c4 = 3.0 / 5.0;
  const double c5 = 1.0;
  const double c6 = 7.0 / 8.0;

  const double b5th1 = 37.0 / 378.0;
  const double b5th2 = 0.0;
  const double b5th3 = 250.0 / 621.0;
  const double b5th4 = 125.0 / 594.0;
  const double b5th5 = 0.0;
  const double b5th6 = 512.0 / 1771.0;

  const double b4th1 = 2825.0 / 27648.0;
  const double b4th2 = 0.0;
  const double b4th3 = 18575.0 / 48384.0;
  const double b4th4 = 13525.0 / 55296.0;
  const double b4th5 = 277.0 / 14336.0;
  const double b4th6 = 1.0 / 4.0;

  const double bDiff1 = b5th1 - b4th1;
  const double bDiff2 = b5th2 - b4th2;
  const double bDiff3 = b5th3 - b4th3;
  const double bDiff4 = b5th4 - b4th4;
  const double bDiff5 = b5th5 - b4th5;
  const double bDiff6 = b5th6 - b4th6;

  const double a21 = 1.0 / 5.0;
  const double a31 = 3.0 / 40.0;
  const double a32 = 9.0 / 40.0;
  const double a41 = 3.0 / 10.0;
  const double a42 = -9.0 / 10.0;
  const double a43 = 6.0 / 5.0;
  const double a51 = -11.0 / 54.0;
  const double a52 = 5.0 / 2.0;
  const double a53 = -70.0 / 27.0;
  const double a54 = 35.0 / 27.0;
  const double a61 = 1631.0 / 55296.0;
  const double a62 = 175.0 / 512.0;
  const double a63 = 575.0 / 13824.0;
  const double a64 = 44275.0 / 110592.0;
  const double a65 = 253.0 / 4096.0;

  const std::size_t stateSize = x.size();
  StateType tempState; // used to store state for next k value and later used
                       // for error difference

  StateType k1;
  dxdt(x, k1, t); // fill k1

  for (std::size_t i = 0; i < stateSize; ++i)
    tempState[i] = x[i] + h * a21 * k1[i];
  StateType k2;
  dxdt(tempState, k2, t + c2 * h); // fill k2

  for (std::size_t i = 0; i < stateSize; ++i)
    tempState[i] = x[i] + h * (a31 * k1[i] + a32 * k2[i]);
  StateType k3;
  dxdt(tempState, k3, t + c3 * h); // fill k3

  for (std::size_t i = 0; i < stateSize; ++i)
    tempState[i] = x[i] + h * (a41 * k1[i] + a42 * k2[i] + a43 * k3[i]);
  StateType k4;
  dxdt(tempState, k4, t + c4 * h); // fill k4

  for (std::size_t i = 0; i < stateSize; ++i)
    tempState[i] =
        x[i] +
        h * (a51 * k1[i] + a52 * k2[i] + a53 * k3[i] + a54 * k4[i]);
  StateType k5;
  dxdt(tempState, k5, t + c5 * h); // fill k5

  for (std::size_t i = 0; i < stateSize; ++i)
    tempState[i] = x[i] +
                   h * (a61 * k1[i] + a62 * k2[i] + a63 * k3[i] +
                        a64 * k4[i] + a65 * k5[i]);
  StateType k6;
  dxdt(tempState, k6, t + c6 * h); // fill k6

  StateType order5Solution;
  for (std::size_t i = 0; i < stateSize; ++i)
    order5Solution[i] =
        h * (b5th1 * k1[i] + b5th2 * k2[i] + b5th3 * k3[i] +
             b5th4 * k4[i] + b5th5 * k5[i] + b5th6 * k6[i]);

  // difference between order 4 and 5, used for error check, reusing tempState
  // variable
  for (std::size_t i = 0; i < stateSize; ++i)
    tempState[i] = h * (bDiff1 * k1[i] + bDiff2 * k2[i] + bDiff3 * k3[i] +
                        bDiff4 * k4[i] + bDiff5 * k5[i] + bDiff6 * k6[i]);

  StateType potentialSolution;
  for (std::size_t i = 0; i < stateSize; ++i)
    potentialSolution[i] = x[i] + order5Solution[i];

  // boost odeint syle error step sizing method
  StateType errorValueList;
  for (std::size_t i = 0; i < stateSize; ++i)
    errorValueList[i] =
        std::abs(tempState[i] / (absTol + relTol * (potentialSolution[i])));
  double maxErrorValue =
      *(std::max_element(errorValueList.begin(), errorValueList.end()));

  // reject step and decrease step size
  if (maxErrorValue > 1.0) {
    h = h * std::max(0.9 * std::pow(maxErrorValue, -0.25), 0.2);
    return 0;
  }

  // use the step
  t += h;
  for (std::size_t i = 0; i < stateSize; ++i)
      x[i] = potentialSolution[i];

  // if error is small enough then increase step size
  if (maxErrorValue < 0.5) {
    h = std::min(h * std::min(0.9 * std::pow(maxErrorValue, -0.20), 5.0),
                 maxStepSize);
  }

  return 1;
}
#endif // CASHKARP54_HPP
