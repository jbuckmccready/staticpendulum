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

#ifndef THREADPOOL_HPP
#define THREADPOOL_HPP
#include <thread>
#include <queue>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <functional>

/* Simple thread pool implementation that accepts void functions
 * with no parameters (uses lambdas to wrap tasks).
 * For a more complex version that utilizes std::future
 * and variadic templates see Jakob Progsch's github project:
 * https://github.com/progschj/ThreadPool
 * However that more complex implementation fails to run under
 * GDB debugger 7.7 on Ubuntu 14.04.
 */

class ThreadPool
{
public:
  ThreadPool(std::size_t threadCount);
  template<typename Function>
  void addTask(Function task);
  ~ThreadPool();
private:
  std::vector<std::thread> workers;
  std::queue<std::function<void()>> taskQueue;
  std::mutex queueMutex;
  std::condition_variable conditionVariable;
  bool stopAllWorkers;
};

inline ThreadPool::ThreadPool(std::size_t threadCount)
  : stopAllWorkers{false}
{
  for(std::size_t i = 0; i < threadCount; ++i)
    workers.emplace_back(
          [this] () // construct the worker for each thread using a lambda
      {
            for(;;)
            {
              std::unique_lock<std::mutex> lock(this->queueMutex);
              while(!this->stopAllWorkers && this->taskQueue.empty())
                this->conditionVariable.wait(lock);
              if(this->stopAllWorkers && this->taskQueue.empty())
                return;
              auto task = this->taskQueue.front();
              this->taskQueue.pop();
              lock.unlock();
              task();
            }
      }
          );
}

template<typename Function>
void ThreadPool::addTask(Function task)
{
  {
    std::unique_lock<std::mutex> lock(queueMutex);
    taskQueue.push(task);
  }
  conditionVariable.notify_one();
}

inline ThreadPool::~ThreadPool()
{
  {
    std::unique_lock<std::mutex> lock(queueMutex);
    stopAllWorkers = true;
  }
  conditionVariable.notify_all();
  for(auto &worker : workers)
    worker.join();
}

#endif // THREADPOOL_HPP
