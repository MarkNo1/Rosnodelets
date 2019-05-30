/*****************************************************************************/
/*!
 *  @file 	 thread_queue.h
 *  @date 	 2019.01.21
 *  @author  Copyright (c) 2018, Will Pearce | Modified by Marco Treglia (AKKA)
 *  @brief 	 Provides a wrapper around a basic queue to provide thread safety.
 *****************************************************************************/

#pragma once

#ifndef ADCC_INTEGRATION_THREAD_QUEUE_H
#define ADCC_INTEGRATION_THREAD_QUEUE_H

/**
 * The ThreadSafeQueue class.
 * Provides a wrapper around a basic queue to provide thread safety.
 */

#include <atomic>
#include <condition_variable>
#include <mutex>
#include <queue>
#include <utility>

namespace renault {
template <typename T>
class ThreadSafeQueue {
 public:
  /**
   * Destructor.
   */
  ~ThreadSafeQueue();

  /**
   * Attempt to get the first value in the queue.
   * Returns true if a value was successfully written to the out parameter, false otherwise.
   */
  bool tryPop(T& out);

  /**
   * Get the first value in the queue.
   * Will block until a value is available unless clear is called or the instance is destructed.
   * Returns true if a value was successfully written to the out parameter, false otherwise.
   */
  bool waitPop(T& out);

  /**
   * Push a new value onto the queue.
   */
  void push(T value);

  /**
   * Check whether or not the queue is empty.
   */
  bool empty() const;

  /**
   * Clear all items from the queue.
   */
  void clear();

  /**
   * Invalidate the queue.
   * Used to ensure no conditions are being waited on in waitPop when
   * a thread or the application is trying to exit.
   * The queue is invalid after calling this method and it is an error
   * to continue using a queue after this method has been called.
   */
  void invalidate();

  /**
   * Returns whether or not this queue is valid.
   */
  bool isValid() const;

 private:
  std::atomic_bool        m_valid{true};
  mutable std::mutex      m_mutex;
  std::queue<T>           m_queue;
  std::condition_variable m_condition;
};
}  // namespace renault

#include <rosnodelets/threads/thread_queue.inl>

#endif  // ADCC_INTEGRATION_THREAD_POOL_H
