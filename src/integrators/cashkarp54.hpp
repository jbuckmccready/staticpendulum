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

/*!
 * \brief Cash and Karp embedded Runge Kutta order 5(4) adaptive step integrator.
 *
 * See: J. R. Cash, A. H. Karp. “A Variable Order Runge-Kutta Method for Initial Value Problems with
 * Rapidly Varying Right-Hand Sides.” ACM Transactions on Mathematical Software, Vol. 16, No. 3, 1990.
 *
 * And Wikipedia: http://en.wikipedia.org/wiki/Cash-Karp_method, http://en.wikipedia.org/wiki/List_of_Runge-Kutta_methods
 */

class CashKarp54
{
public:
  CashKarp54() { }
  template<typename SystemType, typename StateType>
  int operator() (const SystemType &dxdt, StateType &x, double &t, double &h) const;
  void setTolerance(double relativeTolerance, double absoluteTolerance);
  void setMaxStepSize(double maxStepSize);
private:
  // error control parameters
  double m_relativeTolerance = 1e-6;
  double m_absoluteTolerance = 1e-6;
  double m_maxStepSize = 0.1;

  // coefficients for method
  static const double c[6];
  static const double b5th[6];
  static const double b4th[6];
  static const double bDiff[6];
  static const double a[6][5];
};



//! Performs one step for a given state and system, updates the state, time and step size. Returns 1 if successful, 0 if not; in either case udates the step size.
template<typename SystemType, typename StateType>
inline int CashKarp54::operator() (const SystemType &dxdt, StateType &x, double &t, double &h) const
{
  const std::size_t stateSize = x.size();
  StateType k[6];
  StateType tempState; // used to store state for next k value and later used for error difference

  dxdt(x, k[0], t); // fill k1

  for (std::size_t i = 0; i < stateSize; ++i)
    tempState[i] = x[i]+h*a[1][0]*k[0][i];
  dxdt(tempState, k[1], t+c[1]*h); // fill k2

  for (std::size_t i = 0; i < stateSize; ++i)
    tempState[i] = x[i]+h*(a[2][0]*k[0][i]+a[2][1]*k[1][i]);
  dxdt(tempState, k[2], t+c[2]*h); // fill k3

  for (std::size_t i = 0; i < stateSize; ++i)
    tempState[i] = x[i]+h*(a[3][0]*k[0][i]+a[3][1]*k[1][i]+a[3][2]*k[2][i]);
  dxdt(tempState, k[3], t+c[3]*h); // fill k4

  for (std::size_t i = 0; i < stateSize; ++i)
    tempState[i] = x[i]+h*(a[4][0]*k[0][i]+a[4][1]*k[1][i]+a[4][2]*k[2][i]+a[4][3]*k[3][i]);
  dxdt(tempState, k[4], t+c[4]*h); // fill k5

  for (std::size_t i = 0; i < stateSize; ++i)
    tempState[i] = x[i]+h*(a[5][0]*k[0][i]+a[5][1]*k[1][i]+a[5][2]*k[2][i]+a[5][3]*k[3][i]+a[5][4]*k[4][i]);
  dxdt(tempState, k[5], t+c[5]*h); // fill k6

  StateType order5Solution;
  for (std::size_t i = 0; i < stateSize; ++i)
    order5Solution[i] = h*(b5th[0]*k[0][i]+b5th[1]*k[1][i]+b5th[2]*k[2][i]+b5th[3]*k[3][i]+b5th[4]*k[4][i]+b5th[5]*k[5][i]);

  // difference between order 4 and 5, used for error check, reusing tempState variable
  for (std::size_t i = 0; i < stateSize; ++i)
    tempState[i] = h*(bDiff[0]*k[0][i]+bDiff[1]*k[1][i]+bDiff[2]*k[2][i]+bDiff[3]*k[3][i]+bDiff[4]*k[4][i]+bDiff[5]*k[5][i]);

  // boost odeint syle error step sizing method
  StateType errorValueList;
  for (std::size_t i = 0; i < stateSize; ++i)
    errorValueList[i] = std::abs(tempState[i]/(m_absoluteTolerance + m_relativeTolerance * (x[i] + order5Solution[i])));
  double maxErrorValue = *(std::max_element(errorValueList.begin(), errorValueList.end()));

  // reject step and decrease step size
  if (maxErrorValue > 1.0) {
    h = h*std::max(0.9*std::pow(maxErrorValue, -0.25), 0.2);
    return 0;
  }

  // use step and increase step size
  if (maxErrorValue < 0.5) {
    t = t+h;
    for (std::size_t i = 0; i < stateSize; ++i)
      x[i] = x[i] + order5Solution[i];
    h = std::min(h*std::min(0.9*std::pow(maxErrorValue, -0.20), 5.0), m_maxStepSize);
    return 1;
  }

  // else: use step and keep same step size
  t = t+h;
  for (std::size_t i = 0; i < stateSize; ++i)
    x[i] = x[i] + order5Solution[i];
  return 1;


}
#endif // CASHKARP54_HPP
