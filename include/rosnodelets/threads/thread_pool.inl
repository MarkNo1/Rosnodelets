/*****************************************************************************/
/*!
 *  @file 	 thread_task.inl
 *  @date 	 2019.01.21
 *  @author  Copyright (c) 2018, Will Pearce | Modified by Marco Treglia (AKKA)
 *  @brief 	 Threads pool class manager.
 *****************************************************************************/

#include <rosnodelets/threads/thread_pool.h>

namespace renault {

  template<typename Func>
  ThreadPool::ThreadTask<Func>::ThreadTask(Func &&func):m_func{std::move(func)} {
  }


  template<typename Func>
  void ThreadPool::ThreadTask<Func>::execute() {
    m_func();
  }


  template <typename T>
  ThreadPool::TaskFuture<T>::TaskFuture(std::future<T>&& future):m_future{std::move(future)} {
  }


  template <typename T>
  ThreadPool::TaskFuture<T>::~TaskFuture(){
    if(m_future.valid()) {
      m_future.get();
    }
  }

  template <typename T>
  auto ThreadPool::TaskFuture<T>::get() {
      return m_future.get();
  }


  ThreadPool::ThreadPool():ThreadPool{std::max(std::thread::hardware_concurrency(), 2u) - 1u} {
  }

  ThreadPool::ThreadPool(const std::uint32_t numThreads) :m_done{false}, m_workQueue{}, m_threads{} {
    try {
      for(std::uint32_t i = 0u; i < numThreads; ++i) {
        m_threads.emplace_back(&ThreadPool::worker, this);
      }
    }
    catch(...) {
      destroy();
      throw;
    }
}

 ThreadPool::~ThreadPool() {
   destroy();
 }


template <typename Func, typename... Args>
auto ThreadPool::submit(Func&& func, Args&&... args) {
  auto boundTask = std::bind(std::forward<Func>(func), std::forward<Args>(args)...);
  using ResultType = std::result_of_t<decltype(boundTask)()>;
  using PackagedTask = std::packaged_task<ResultType()>;
  using TaskType = ThreadTask<PackagedTask>;

  PackagedTask task{std::move(boundTask)};
  TaskFuture<ResultType> result{task.get_future()};
  m_workQueue.push(std::make_unique<TaskType>(std::move(task)));
  return result;
}

void ThreadPool::worker() {
  while(!m_done) {
    std::unique_ptr<IThreadTask> pTask{nullptr};
    if(m_workQueue.waitPop(pTask)) {
      pTask->execute();
    }
  }
}

void ThreadPool::destroy() {
  m_done = true;
  m_workQueue.invalidate();
  for(auto& thread : m_threads) {
    if(thread.joinable()) {
      thread.join();
    }
  }
}

inline ThreadPool& DefaultThreadPool::getThreadPool()  {
  static ThreadPool defaultPool;
  return defaultPool;
}


template <typename Func, typename... Args>
inline auto DefaultThreadPool::submitJob(Func&& func, Args&&... args) {
  return getThreadPool().submit(std::forward<Func>(func), std::forward<Args>(args)...);
}


}