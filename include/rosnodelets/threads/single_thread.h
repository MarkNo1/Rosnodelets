/*****************************************************************************/
/*!
 *  @file 	 thread_task.h
 *  @date 	 2019.02.19
 *  @author  Marco Treglia (AKKA)
 *  @brief 	 Single Thread Class
 *****************************************************************************/

#ifndef ADCC_INTEGRATION_SINGLE_THREAD_H
#define ADCC_INTEGRATION_SINGLE_THREAD_H

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

/**
 *  @class  CThread
 *  @brief  Template class to compute a task using in a thread.
 */
class CThread {
 public:
  /**
   *  Constructor - To override.
   */
  CThread() : finished_(false){};

  /**
   *  Destructor.
   */
  ~CThread() {
    Terminate();
    if (thread_->joinable()) Join();
  };

  /**
   *  Start the task function inside a Thread.
   */
  virtual void Start() { thread_ = boost::make_shared<Thread>(boost::bind(&CThread::ThreadTask, this)); };

  /**
   *  join the thread.
   */
  void Join() { thread_->join(); };

  /**
   *  Terminate the thread with interrupt.
   */
  void Terminate() { thread_->interrupt(); };

  /**
   *  Task to compute inside a Thread.
   */
  virtual bool Task() = 0;

  /**
   *  Task called by the Thread
   */
  void ThreadTask() { SetFinished(Task()); };

  /**
   * Status of the Thread
   * @return  status
   */
  bool isFinished() {
    Lock guard(finished_mutex_);
    return finished_;
  };

  using Thread    = boost::thread;
  using ThreadPtr = boost::shared_ptr<Thread>;
  using Ptr       = boost::shared_ptr<CThread>;
  using Mutex     = boost::mutex;
  using Lock      = boost::lock_guard<Mutex>;

 protected:
  /**
   *  Set finished_ variable
   * @param value
   */
  void SetFinished(bool value) {
    Lock guard(finished_mutex_);
    finished_ = value;
  };

  ThreadPtr thread_;
  Mutex     finished_mutex_;
  bool      finished_;
};

#endif  // ADCC_INTEGRATION_SINGLE_THREAD_H
