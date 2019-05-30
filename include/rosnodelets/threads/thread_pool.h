/*****************************************************************************/
/*!
 *  @file 	 thread_task.h
 *  @date 	 2019.01.21
 *  @author  Copyright (c) 2018, Will Pearce | Modified by Marco Treglia (AKKA)
 *  @brief 	 Threads pool class manager.
 *****************************************************************************/

#pragma once

#ifndef ADCC_INTEGRATION_THREAD_TASK_H
#define ADCC_INTEGRATION_THREAD_TASK_H

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <functional>
#include <future>
#include <memory>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

#include <rosnodelets/threads/thread_queue.h>

namespace renault {

class ThreadPool {
 private:
  class IThreadTask {
   public:
    IThreadTask()                       = default;
    virtual ~IThreadTask()              = default;
    IThreadTask(const IThreadTask& rhs) = delete;
    IThreadTask& operator=(const IThreadTask& rhs) = delete;
    IThreadTask(IThreadTask&& other)               = default;
    IThreadTask& operator=(IThreadTask&& other) = default;

    /**
     * Run the task.
     */
    virtual void execute() = 0;
  };

  template <typename Func>
  class ThreadTask : public IThreadTask {
   public:
    explicit ThreadTask(Func&& func);

    ~ThreadTask() override            = default;
    ThreadTask(const ThreadTask& rhs) = delete;
    ThreadTask& operator=(const ThreadTask& rhs) = delete;
    ThreadTask(ThreadTask&& other) noexcept      = default;
    ThreadTask& operator=(ThreadTask&& other) noexcept = default;

    /**
     * Run the task.
     */
    void execute() override;

   private:
    Func m_func;
  };

 public:
  /**
   * A wrapper around a std::future that adds the behavior of futures returned from std::async.
   * Specifically, this object will block and wait for execution to finish before going out of scope.
   */
  template <typename T>
  class TaskFuture {
   public:
    explicit TaskFuture(std::future<T>&& future);

    TaskFuture(const TaskFuture& rhs) = delete;
    TaskFuture& operator=(const TaskFuture& rhs) = delete;
    TaskFuture(TaskFuture&& other) noexcept      = default;
    TaskFuture& operator=(TaskFuture&& other) noexcept = default;
    ~TaskFuture();

    auto get();

   private:
    std::future<T> m_future;
  };

 public:
  /**
   * Constructor.
   *
   * Always create at least one thread.  If hardware_concurrency() returns 0,
   * subtracting one would turn it to UINT_MAX, so get the maximum of
   * hardware_concurrency() and 2 before subtracting 1.
   */
  ThreadPool();

  /**
   * Constructor.
   */
  explicit ThreadPool(const std::uint32_t numThreads);

  /**
   * Non-copyable.
   */
  ThreadPool(const ThreadPool& rhs) = delete;

  /**
   * Non-assignable.
   */
  ThreadPool& operator=(const ThreadPool& rhs) = delete;

  /**
   * Destructor.
   */
  ~ThreadPool();

  /**
   * Submit a job to be run by the thread pool.
   */
  template <typename Func, typename... Args>
  auto submit(Func&& func, Args&&... args);

 private:
  /**
   * Constantly running function each thread uses to acquire work items from the queue.
   */
  void worker();

  /**
   * Invalidates the queue and joins all running threads.
   */
  void destroy();

 private:
  std::atomic_bool                              m_done;
  ThreadSafeQueue<std::unique_ptr<IThreadTask>> m_workQueue;
  std::vector<std::thread>                      m_threads;
};

namespace DefaultThreadPool {
  /**
   * Get the default thread pool for the application.
   * This pool is created with std::thread::hardware_concurrency() - 1 threads.
   */
  inline ThreadPool& getThreadPool();

  /**
   * Submit a job to the default thread pool.
   */
  template <typename Func, typename... Args>
  inline auto submitJob(Func&& func, Args&&... args);

}  // namespace DefaultThreadPool

}  // namespace renault

#include <rosnodelets/threads/thread_pool.inl>

#endif  // ADCC_INTEGRATION_THREAD_TASK_H
