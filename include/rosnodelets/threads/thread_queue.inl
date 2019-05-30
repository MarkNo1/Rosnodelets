/*****************************************************************************/
/*!
 *  @file 	 thread_queue.h
 *  @date 	 2019.01.21
 *  @author  Copyright (c) 2018, Will Pearce | Modified by Marco Treglia (AKKA)
 *  @brief 	 Provides a wrapper around a basic queue to provide thread safety.
 *****************************************************************************/

#include <rosnodelets/threads/thread_queue.h>

namespace renault
{

  /**
   * Destructor.
   */
  template <typename T>
  ThreadSafeQueue<T>::~ThreadSafeQueue(void)
  {
    invalidate();
  }

  /**
   * Attempt to get the first value in the queue.
   * Returns true if a value was successfully written to the out parameter, false otherwise.
   */
  template <typename T>
  bool ThreadSafeQueue<T>::tryPop(T& out) {
    std::lock_guard<std::mutex> lock{m_mutex};
    if(m_queue.empty() || !m_valid)
    {
      return false;
    }
    out = std::move(m_queue.front());
    m_queue.pop();
    return true;
  }

  /**
   * Get the first value in the queue.
   * Will block until a value is available unless clear is called or the instance is destructed.
   * Returns true if a value was successfully written to the out parameter, false otherwise.
   */
  template <typename T>
  bool ThreadSafeQueue<T>::waitPop(T& out) {
    std::unique_lock<std::mutex> lock{m_mutex};
    m_condition.wait(lock, [this]() {
      return !m_queue.empty() || !m_valid;
    });
    /*
     * Using the condition in the predicate ensures that spurious wake-ups with a valid
     * but empty queue will not proceed, so only need to check for validity before proceeding.
     */
    if(!m_valid) {
      return false;
    }
    out = std::move(m_queue.front());
    m_queue.pop();
    return true;
  }

  /**
   * Push a new value onto the queue.
   */
  template <typename T>
  void ThreadSafeQueue<T>::push(T value) {
    std::lock_guard<std::mutex> lock{m_mutex};
    m_queue.push(std::move(value));
    m_condition.notify_one();
  }

  /**
   * Check whether or not the queue is empty.
   */
  template <typename T>
  bool ThreadSafeQueue<T>::empty(void) const {
    std::lock_guard<std::mutex> lock{m_mutex};
    return m_queue.empty();
  }

  /**
   * Clear all items from the queue.
   */
  template <typename T>
  void ThreadSafeQueue<T>::clear(void) {
    std::lock_guard<std::mutex> lock{m_mutex};
    while(!m_queue.empty()) {
      m_queue.pop();
    }
    m_condition.notify_all();
  }

  /**
   * Invalidate the queue.
   * Used to ensure no conditions are being waited on in waitPop when
   * a thread or the application is trying to exit.
   * The queue is invalid after calling this method and it is an error
   * to continue using a queue after this method has been called.
   */
  template <typename T>
  void ThreadSafeQueue<T>::invalidate(void) {
    std::lock_guard<std::mutex> lock{m_mutex};
    m_valid = false;
    m_condition.notify_all();
  }

  /**
   * Returns whether or not this queue is valid.
   */
  template <typename T>
  bool ThreadSafeQueue<T>::isValid(void) const {
    std::lock_guard<std::mutex> lock{m_mutex};
    return m_valid;
  }

}
