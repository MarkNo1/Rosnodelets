/*****************************************************************************/
/*!
 *  @file 	 BaseNodelet.h
 *  @date 	 2019.01.09
 *  @author  M. Treglia Akka
 *  @brief 	 Base class used by the nodelets.
 *****************************************************************************/

#ifndef PROJECT_NODELET_H
#define PROJECT_NODELET_H

#include <chrono>
#include <sstream>

#include <ros/ros.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>


namespace base {

/**
 *  @class  BaseNodelet
 *  @brief  Template for the nodelet implementation.
 */
class Nodelet : public nodelet::Nodelet {
 public:
  using nodelet::Nodelet::Nodelet;

  /**
   *  @brief Method called by the nodelet to initialize itself.
   */
  void onInit() override {
    InitializeNodeletName();
    LoadRosParameters();
    InitializeNodelet();
    EstablishRosCommunication();
  }

  /**
   * Initialize the nodelet name
   */
  virtual void InitializeNodeletName() = 0;
  /**
   * @brief Virtual. Loads all the ros parameters.
   */
  virtual void LoadRosParameters() = 0;

  /**
   * @brief Virtual. Initialize all nodelets components.
   */
  virtual void InitializeNodelet() = 0;

  /**
   * @brief Virtual. Open ros topic communications. (Publishers, Subscribers)
   */
  virtual void EstablishRosCommunication() = 0;

  /**
   * Get nodelet name
   * @return name
   */
  const std::string& GetName() const { return name_; }

  /**
   * Get nodelet name str
   * @return name
   */
  const char* GetNameStr() const { return name_.c_str(); }

  /**
   * Set nodelet name
   * @param name
   */
  void SetName(const std::string& name) { name_ = name; }

  /**
   * Print log info with char*
   * @param text
   */
  void LogInfo(const char* text) { NODELET_INFO("[%s] - %s", GetNameStr(), text); };

  /**
   * Log info with stream
   * @param stream
   */
  void LogInfo(const std::string& text) { NODELET_INFO_STREAM("[" << GetName() << "] - " << text); }

 private:
  std::string name_;
};

// Using chrono
using Time        = std::chrono::system_clock;
using Microsecond = std::chrono::microseconds;
using Millisecond = std::chrono::milliseconds;

/**
 *  @class  NodeletBenchMark
 *  @brief  Template for the nodelet implementation with the benchmarker tool.
 */
class NodeletBM : public Nodelet {
 public:
  using Nodelet::Nodelet;

 protected:
  /**
   *  @brief Method called by the nodelet to initialize itself.
   */
  void onInit() final {
    InitializeNodeletName();
    LoadRosParameters();
    InitializeNodelet();
    EstablishRosCommunication();
  }

  /**
   *  Start timer
   * @param value bool
   */
  void Start() { start_ = Time::now(); };

  /**
   * Get elapsed time
   * @return bool
   */
  double Duration() { return std::chrono::duration_cast<Millisecond>(Time::now() - start_).count(); }

  /**
   * End timer and print time.
   */
  void End() {
    double elapsed(Duration());
    LogInfo(Cast<double>(elapsed) + " ms");
  }

  template <typename T>
  const std::string Cast(T& val) {
    return boost::lexical_cast<std::string>(val);
  };

 private:
  Time::time_point start_;
};

}  // namespace base

#endif  // PROJECT_NODELET_H
