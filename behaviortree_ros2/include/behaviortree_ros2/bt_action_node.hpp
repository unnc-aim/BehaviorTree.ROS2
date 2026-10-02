// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2023 Davide Faconti
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <memory>
#include <string>
#include <algorithm>  // std::remove_if 用于移除已经结束的取消请求。
#include <chrono>     // steady_clock 使用单调时间计算响应期限。
#include <vector>     // vector 保存本次收到的反馈及待取消请求。
#include <rclcpp/executors.hpp>
#include <rclcpp/allocator/allocator_common.hpp>
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp_action/rclcpp_action.hpp"

#include "behaviortree_ros2/ros_node_params.hpp"

namespace BT
{

enum ActionNodeErrorCode
{
  SERVER_UNREACHABLE,
  SEND_GOAL_TIMEOUT,
  GOAL_REJECTED_BY_SERVER,
  ACTION_ABORTED,
  ACTION_CANCELLED,
  INVALID_GOAL
};

inline const char* toStr(const ActionNodeErrorCode& err)
{
  switch(err)
  {
    case SERVER_UNREACHABLE:
      return "SERVER_UNREACHABLE";
    case SEND_GOAL_TIMEOUT:
      return "SEND_GOAL_TIMEOUT";
    case GOAL_REJECTED_BY_SERVER:
      return "GOAL_REJECTED_BY_SERVER";
    case ACTION_ABORTED:
      return "ACTION_ABORTED";
    case ACTION_CANCELLED:
      return "ACTION_CANCELLED";
    case INVALID_GOAL:
      return "INVALID_GOAL";
  }
  return nullptr;
}

/**
 * @brief Abstract class to wrap rclcpp_action::Client<>
 *
 * For instance, given the type AddTwoInts described in this tutorial:
 * https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Cpp.html
 *
 * the corresponding wrapper would be:
 *
 * class FibonacciNode: public RosActionNode<action_tutorials_interfaces::action::Fibonacci>
 *
 * RosActionNode will try to be non-blocking for the entire duration of the call.
 * The derived class must reimplement the virtual methods as described below.
 *
 * The name of the action will be determined as follows:
 *
 * 1. If a value is passes in the InputPort "action_name", use that
 * 2. Otherwise, use the value in RosNodeParams::default_port_value
 */
template <class ActionT>
class RosActionNode : public BT::ActionNodeBase
{
public:
  // Type definitions
  using ActionType = ActionT;
  using ActionClient = typename rclcpp_action::Client<ActionT>;
  using ActionClientPtr = std::shared_ptr<ActionClient>;
  using Goal = typename ActionT::Goal;
  using GoalHandle = typename rclcpp_action::ClientGoalHandle<ActionT>;
  using WrappedResult = typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult;
  using Feedback = typename ActionT::Feedback;

  /** To register this class into the factory, use:
   *
   *    factory.registerNodeType<>(node_name, params);
   *
   */
  explicit RosActionNode(const std::string& instance_name, const BT::NodeConfig& conf,
                         const RosNodeParams& params);

  ~RosActionNode() override;  // override 表示覆盖虚析构；销毁时交接未完成目标。

  /**
   * @brief Any subclass of RosActionNode that has ports must implement a
   * providedPorts method and call providedBasicPorts in it.
   *
   * @param addition Additional ports to add to BT port list
   * @return PortsList containing basic ports along with node-specific ports
   */
  static PortsList providedBasicPorts(PortsList addition)
  {
    PortsList basic = { InputPort<std::string>("action_name", "", "Action server name") };
    basic.insert(addition.begin(), addition.end());
    return basic;
  }

  /**
   * @brief Creates list of BT ports
   * @return PortsList Containing basic ports along with node-specific ports
   */
  static PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

  /// @brief  Callback executed when the node is halted. Note that cancelGoal()
  /// is done automatically.
  virtual void onHalt()
  {}

  /** setGoal s a callback that allows the user to set
   *  the goal message (ActionT::Goal).
   *
   * @param goal  the goal to be sent to the action server.
   *
   * @return false if the request should not be sent. In that case,
   * RosActionNode::onFailure(INVALID_GOAL) will be called.
   */
  virtual bool setGoal(Goal& goal) = 0;

  /** Callback invoked when the result is received by the server.
   * It is up to the user to define if the action returns SUCCESS or FAILURE.
   */
  virtual BT::NodeStatus onResultReceived(const WrappedResult& result) = 0;

  /** Callback invoked when the feedback is received.
   * It generally returns RUNNING, but the user can also use this callback to cancel the
   * current action and return SUCCESS or FAILURE.
   */
  virtual BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> /*feedback*/)
  {
    return NodeStatus::RUNNING;
  }

  /** Callback invoked when something goes wrong.
   * It must return either SUCCESS or FAILURE.
   */
  virtual BT::NodeStatus onFailure(ActionNodeErrorCode /*error*/)
  {
    return NodeStatus::FAILURE;
  }

  /// 请求取消当前目标并立即返回；父 ROS 执行器继续处理接受响应和最终结果。
  /// 手动调用 tick 的程序也应持续执行父节点回调，以便 halt 后完成清理。
  void cancelGoal();

  /// The default halt() implementation will call cancelGoal if necessary.
  void halt() override;

  NodeStatus tick() override;

  /// Can be used to change the name of the action programmatically
  void setActionName(const std::string& action_name);

protected:
  // 每次发送独立保存状态；ROS 回调只访问这份状态，避免引用已销毁的 BT 节点。
  struct RequestState
  {
    typename GoalHandle::SharedPtr handle;  // typename 指明依赖模板参数的句柄类型。
    WrappedResult result{};                // {} 将结果初始化为 UNKNOWN。
    std::vector<std::shared_ptr<const Feedback>> feedback;  // 保留反馈顺序，供 tick 调用虚函数。
    std::chrono::steady_clock::time_point sent_at;  // 单调时间记录本次发送时刻。
    NodeStatus feedback_status = NodeStatus::RUNNING;  // 保存 onFeedback 请求的结束状态。
    bool response_received = false;  // 接受或拒绝响应均设置为 true。
    bool timed_out = false;          // 记录本次响应等待已经超时。
    bool cancel_requested = false;  // 即使句柄尚未到达，也保留取消意图。
    bool cancel_sent = false;       // 每个目标只提交一次取消请求。
    bool finished = false;          // 收到拒绝或最终结果后结束本次请求。
  };

  // enable_shared_from_this 允许取消定时器在 BT 节点结束后继续持有客户端。
  struct ActionClientInstance : std::enable_shared_from_this<ActionClientInstance>
  {
    ActionClientInstance(std::shared_ptr<rclcpp::Node> node,
                         const std::string& action_name);

    ActionClientPtr action_client;
    rclcpp::CallbackGroup::SharedPtr callback_group;
    rclcpp::executors::SingleThreadedExecutor callback_executor;
    std::weak_ptr<rclcpp::Node> node;  // weak_ptr 平时只观察父 ROS 节点的生命周期。
    std::vector<std::shared_ptr<RequestState>> pending_cancellations;  // 保留尚未结束的取消请求。
    rclcpp::TimerBase::SharedPtr cleanup_timer;  // 父执行器负责在停树后继续处理取消。

    void poll();  // 调用者持有 getMutex()；按 5ms 预算处理回调，当前回调会执行完。
    void cancel(const std::shared_ptr<RequestState>& request);  // 登记本次请求并启动清理。
    void sendCancel(const std::shared_ptr<RequestState>& request);  // 只取消该请求的目标。
  };

  static std::mutex& getMutex()
  {
    static std::mutex action_client_mutex;
    return action_client_mutex;
  }

  rclcpp::Logger logger()
  {
    if(auto node = node_.lock())
    {
      return node->get_logger();
    }
    return rclcpp::get_logger("RosActionNode");
  }

  rclcpp::Time now()
  {
    if(auto node = node_.lock())
    {
      return node->now();
    }
    return rclcpp::Clock(RCL_ROS_TIME).now();
  }

  using ClientsRegistry =
      std::unordered_map<std::string, std::weak_ptr<ActionClientInstance>>;
  // contains the fully-qualified name of the node and the name of the client
  static ClientsRegistry& getRegistry()
  {
    static ClientsRegistry action_clients_registry;
    return action_clients_registry;
  }

  std::weak_ptr<rclcpp::Node> node_;
  std::shared_ptr<ActionClientInstance> client_instance_;
  std::string action_name_;
  bool action_name_should_be_checked_ = false;
  const std::chrono::milliseconds server_timeout_;
  const std::chrono::milliseconds wait_for_server_timeout_;
  std::string action_client_key_;

private:
  std::shared_ptr<RequestState> request_;  // 当前尝试持有自己的句柄、反馈和结果。

  bool createClient(const std::string& action_name);
};

//----------------------------------------------------------------
//---------------------- DEFINITIONS -----------------------------
//----------------------------------------------------------------

template <class T>
RosActionNode<T>::ActionClientInstance::ActionClientInstance(
    std::shared_ptr<rclcpp::Node> parent, const std::string& action_name)
  : node(parent)  // 初始化列表保存父节点的弱引用。
{
  callback_group =
      parent->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);  // false 交由专用执行器管理。
  callback_executor.add_callback_group(callback_group, parent->get_node_base_interface());  // 显式注册回调组。
  action_client = rclcpp_action::create_client<T>(parent, action_name, callback_group);  // 复用已有 Action 客户端接口。
}

template <class T>
void RosActionNode<T>::ActionClientInstance::poll()
{
  callback_executor.spin_all(std::chrono::milliseconds(5));  // 反复取回调；队列空时立即返回。
  for(const auto& request : pending_cancellations)  // const auto& 避免复制共享指针。
  {
    sendCancel(request);  // 句柄刚到达时补发取消，已经发送的请求直接跳过。
  }
  pending_cancellations.erase(  // erase 配合 remove_if 真正删除结束的请求。
      std::remove_if(pending_cancellations.begin(), pending_cancellations.end(),
                     [](const auto& request) { return request->finished; }),
      pending_cancellations.end());
  if(pending_cancellations.empty() && cleanup_timer)  // 清理完成后释放定时器持有的客户端。
  {
    cleanup_timer->cancel();  // 停止后续定时调用。
    cleanup_timer.reset();    // reset 释放 shared_ptr，解除定时器与客户端的相互持有。
  }
}

template <class T>
void RosActionNode<T>::ActionClientInstance::sendCancel(
    const std::shared_ptr<RequestState>& request)
{
  if(request->finished || request->cancel_sent || !request->handle)  // 等待句柄，或复用已有取消请求。
  {
    return;  // 本轮没有新的取消操作。
  }
  auto parent = node.lock();  // 发送期间临时持有父节点。
  if(!parent || !rclcpp::ok(parent->get_node_base_interface()->get_context()))
  {
    return;  // ROS 退出后交由关闭回调释放清理状态。
  }
  try
  {
    const auto logger = parent->get_logger();  // 日志器按值捕获，无需引用客户端对象。
    action_client->async_cancel_goal(request->handle, [logger](const auto& response) {
      if(response->return_code != ActionClient::CancelResponse::ERROR_NONE)
      {
        RCLCPP_WARN(logger, "Goal cancellation returned %d; waiting for terminal result",
                    response->return_code);  // 取消被拒绝时继续保留请求，等待明确终态。
      }
    });  // 按目标句柄取消，范围限于本次请求。
    request->cancel_sent = true;  // 最终结束由 result_callback 确认。
  }
  catch(const rclcpp_action::exceptions::UnknownGoalHandleError&)
  {
    request->finished = true;  // 客户端已移除该句柄，按已结束处理。
  }
  catch(const rclcpp::exceptions::RCLError& error)
  {
    RCLCPP_ERROR_THROTTLE(parent->get_logger(), *parent->get_clock(), 2000,
                         "Goal cancellation send failed: %s", error.what());  // 保留请求，后续 poll 再试。
  }
}

template <class T>
void RosActionNode<T>::ActionClientInstance::cancel(
    const std::shared_ptr<RequestState>& request)
{
  if(!request || request->finished || request->cancel_requested)  // 接受空请求及重复取消调用。
  {
    return;  // 已完成或已经登记时保持原状态。
  }
  request->cancel_requested = true;  // 先记录意图，覆盖接受响应迟到的情况。
  request->feedback.clear();        // 取消期间只处理响应和最终结果。
  pending_cancellations.push_back(request);  // 客户端接管状态，支持 BT 节点销毁。
  auto parent = node.lock();  // lock 将仍存活的 weak_ptr 临时提升为 shared_ptr。
  if(!parent || !rclcpp::ok(parent->get_node_base_interface()->get_context()))
  {
    return;  // ROS 已结束时保留取消记录，跳过新的 ROS 操作。
  }
  if(!cleanup_timer)  // 同一个客户端复用一个清理定时器。
  {
    auto self = this->shared_from_this();  // 定时器保持客户端存活直到目标结束。
    cleanup_timer = parent->create_wall_timer(std::chrono::milliseconds(10), [self, parent]() {  // 按值捕获 shared_ptr 保持两者存活。
      std::unique_lock<std::mutex> lock(getMutex(), std::try_to_lock);  // 与树线程串行处理回调。
      if(lock.owns_lock() && rclcpp::ok(parent->get_node_base_interface()->get_context()))
      {
        try
        {
          self->poll();  // 停树后仍由父 ROS 执行器处理迟到响应及取消结果。
        }
        catch(const rclcpp::exceptions::RCLError& error)
        {
          if(rclcpp::ok(parent->get_node_base_interface()->get_context()))
          {
            RCLCPP_ERROR_THROTTLE(parent->get_logger(), *parent->get_clock(), 2000,
                                 "Action cleanup callback failed: %s", error.what());  // 活跃上下文保留真实错误。
          }
        }
      }
    });
  }
  sendCancel(request);  // 已有句柄时立即发送；其余情况由后续 poll 继续。
}

template <class T>
inline RosActionNode<T>::RosActionNode(const std::string& instance_name,
                                       const NodeConfig& conf,
                                       const RosNodeParams& params)
  : BT::ActionNodeBase(instance_name, conf)
  , node_(params.nh)
  , server_timeout_(params.server_timeout)
  , wait_for_server_timeout_(params.wait_for_server_timeout)
{
  // Three cases:
  // - we use the default action_name in RosNodeParams when port is empty
  // - we use the action_name in the port and it is a static string.
  // - we use the action_name in the port and it is blackboard entry.

  // check port remapping
  auto portIt = config().input_ports.find("action_name");
  if(portIt != config().input_ports.end())
  {
    const std::string& bb_service_name = portIt->second;

    if(isBlackboardPointer(bb_service_name))
    {
      // unknown value at construction time. Postpone to tick
      action_name_should_be_checked_ = true;
    }
    else if(!bb_service_name.empty())
    {
      // "hard-coded" name in the bb_service_name. Use it.
      createClient(bb_service_name);
    }
  }
  // no port value or it is empty. Use the default value
  if(!client_instance_ && !params.default_port_value.empty())
  {
    createClient(params.default_port_value);
  }
}

template <class T>
inline bool RosActionNode<T>::createClient(const std::string& action_name)
{
  if(action_name.empty())
  {
    throw RuntimeError("action_name is empty");
  }

  std::unique_lock lk(getMutex());
  auto node = node_.lock();
  if(!node)
  {
    throw RuntimeError("The ROS node went out of scope. RosNodeParams doesn't take the "
                       "ownership of the node.");
  }
  action_client_key_ = std::string(node->get_fully_qualified_name()) + "/" + action_name;

  auto& registry = getRegistry();
  auto it = registry.find(action_client_key_);
  client_instance_ = it == registry.end() ? nullptr : it->second.lock();  // 尝试取得仍存活的缓存。
  if(!client_instance_ || client_instance_->node.lock() != node)  // 同名的不同 ROS 节点各用自己的客户端。
  {
    client_instance_ = std::make_shared<ActionClientInstance>(node, action_name);
    registry[action_client_key_] = client_instance_;  // 覆盖过期条目，使后续节点复用新客户端。
    std::weak_ptr<ActionClientInstance> weak_client = client_instance_;  // 关闭回调采用弱引用。
    node->get_node_base_interface()->get_context()->on_shutdown([weak_client]() {
      std::lock_guard<std::mutex> lock(getMutex());  // 与 tick 和取消定时器保护同一份状态。
      if(auto client = weak_client.lock())  // if 初始化语句只在客户端仍存活时进入。
      {
        if(client->cleanup_timer)  // ROS 关闭时释放尚未完成的定时器持有关系。
        {
          client->cleanup_timer->cancel();  // 停止清理定时器。
          client->cleanup_timer.reset();    // 释放定时器及其捕获的 shared_ptr。
        }
        client->pending_cancellations.clear();  // 释放待处理请求；ROS 回调仅持有弱引用。
      }
    });
  }

  action_name_ = action_name;

  bool found =
      client_instance_->action_client->wait_for_action_server(wait_for_server_timeout_);
  if(!found)
  {
    RCLCPP_ERROR(logger(), "%s: Action server with name '%s' is not reachable.",
                 name().c_str(), action_name_.c_str());
  }
  return found;
}

template <class T>
inline void RosActionNode<T>::setActionName(const std::string& action_name)
{
  if(action_name == action_name_)  // 同一个名称无需替换客户端。
  {
    return;
  }
  {
    std::lock_guard<std::mutex> lock(getMutex());  // 切换前检查旧客户端的请求状态。
    if((request_ && !request_->finished) ||
       (client_instance_ && !client_instance_->pending_cancellations.empty()))
    {
      throw LogicError("setActionName requires the previous goal to finish");  // 保持旧句柄与旧客户端对应。
    }
    request_.reset();  // 已结束请求随名称切换一起释放。
  }
  action_name_ = action_name;
  createClient(action_name);
}

// 析构只交接取消；ROS 回调通过弱引用访问请求，避免调用已析构对象的虚函数。
template <class T>
RosActionNode<T>::~RosActionNode()
{
  try
  {
    cancelGoal();  // 与 halt 共用定向取消处理，父 ROS 执行器继续完成清理。
  }
  catch(const std::exception& error)
  {
    RCLCPP_ERROR(logger(), "Action cleanup during destruction failed: %s", error.what());  // 析构函数记录异常后正常返回。
  }
}

template <class T>
inline NodeStatus RosActionNode<T>::tick()
{
  auto parent = node_.lock();  // 临时持有节点，确保本次 tick 内节点存活。
  if(!parent || !rclcpp::ok(parent->get_node_base_interface()->get_context()))
  {
    cancelGoal();  // ROS 退出时登记取消状态，跳过新的目标发送。
    return NodeStatus::FAILURE;  // 通知上层结束本次执行。
  }

  if(!client_instance_ || (status() == NodeStatus::IDLE && action_name_should_be_checked_))
  {
    std::string action_name;  // 保存端口当前指定的 Action 名称。
    getInput("action_name", action_name);  // 支持黑板中的名称在两次运行之间变化。
    if(action_name_ != action_name || !client_instance_)
    {
      createClient(action_name);  // createClient 自行加锁，因此放在下方锁之前。
    }
  }
  if(!client_instance_)
  {
    throw BT::RuntimeError("RosActionNode: no client was specified");  // 缺少客户端时报告配置错误。
  }

  auto check_status = [](NodeStatus value) {  // lambda 统一检查派生类返回的结束状态。
    if(!isStatusCompleted(value))
    {
      throw LogicError("RosActionNode: the callback must return SUCCESS or FAILURE");
    }
    return value;  // 返回已经检查的 SUCCESS 或 FAILURE。
  };

  std::unique_lock<std::mutex> lock(getMutex());  // 与取消定时器串行访问客户端和请求状态。
  client_instance_->poll();  // 使用 spin_all(5ms)，及时处理反馈后的响应和结果。
  if(status() == NodeStatus::IDLE)
  {
    request_.reset();  // 释放上一轮的已结束状态；待取消状态仍由客户端保存。
    setStatus(NodeStatus::RUNNING);  // 等待旧目标清理时，上层仍可检查比赛结束条件。
  }

  if(!request_)
  {
    if(!client_instance_->pending_cancellations.empty())
    {
      return NodeStatus::RUNNING;  // 同一 Action 的旧请求结束后，再发送新的目标。
    }
    Goal goal;  // 派生类负责填充具体 Action 的目标字段。
    lock.unlock();  // 虚函数可能调用 setActionName，调用前释放客户端锁。
    const bool valid_goal = setGoal(goal);  // 沿用派生类的目标构造接口。
    lock.lock();  // 访问客户端状态前重新取得同一把锁。
    if(!valid_goal)
    {
      return check_status(onFailure(INVALID_GOAL));  // 保留原有无效目标处理接口。
    }
    if(!client_instance_->action_client->action_server_is_ready())
    {
      return check_status(onFailure(SERVER_UNREACHABLE));  // 服务端就绪后由上层重试。
    }

    request_ = std::make_shared<RequestState>();  // 每次发送创建独立状态，初始化空句柄和 UNKNOWN 结果。
    request_->sent_at = std::chrono::steady_clock::now();  // 响应计时使用单调时钟。
    std::weak_ptr<RequestState> weak_request = request_;  // 回调观察本次请求，避免与句柄形成循环持有。
    std::weak_ptr<ActionClientInstance> weak_client = client_instance_;  // 迟到接受需要访问对应客户端。
    typename ActionClient::SendGoalOptions options;  // typename 指明依赖 T 的嵌套类型。

    options.goal_response_callback = [weak_request, weak_client](typename GoalHandle::SharedPtr handle) {  // lambda 按值捕获两份弱引用。
      if(auto request = weak_request.lock())  // 本次请求存活时才更新状态。
      {
        request->handle = handle;  // 立即保存 shared_ptr；结果回调随后即可核对目标。
        request->response_received = true;  // 空句柄表示服务端拒绝目标。
        request->finished = !handle;  // 拒绝目标已经结束，无需发送取消。
        if(handle && request->cancel_requested)
        {
          if(auto client = weak_client.lock())
          {
            client->sendCancel(request);  // 接受响应迟到时，按这个句柄补发取消。
          }
        }
      }
    };
    options.feedback_callback = [weak_request](  // 每个回调绑定发送时的请求状态。
        typename GoalHandle::SharedPtr handle, const std::shared_ptr<const Feedback> feedback) {  // const Feedback 表示只读消息。
      if(auto request = weak_request.lock())
      {
        if(!request->cancel_requested && !request->finished && request->handle && handle &&
           request->handle->get_goal_id() == handle->get_goal_id())  // 核对反馈所属目标。
        {
          request->feedback.push_back(feedback);  // tick 按顺序调用当前节点的 onFeedback。
        }
      }
    };
    options.result_callback = [weak_request](const WrappedResult& result) {  // const 引用读取结果，避免复制回调入参。
      if(auto request = weak_request.lock())
      {
        if(request->handle && request->handle->get_goal_id() == result.goal_id)  // 先判空再核对目标。
        {
          request->result = result;  // 旧请求的结果只写回旧请求自己的状态。
          request->finished = true;  // 最终结果确认目标已经结束。
        }
      }
    };

    try
    {
      client_instance_->action_client->async_send_goal(goal, options);  // 接受回调接管句柄，无需另存 future。
    }
    catch(...)
    {
      request_->finished = true;  // 发送调用失败时结束本地状态，避免进入等待接受响应的清理循环。
      throw;  // 将原异常继续交给行为树执行服务器处理。
    }
    return NodeStatus::RUNNING;  // 下一次 tick 检查接受响应和执行结果。
  }

  if(!request_->response_received && !request_->timed_out &&
     std::chrono::steady_clock::now() - request_->sent_at > server_timeout_)
  {
    request_->timed_out = true;  // 记录超时原因，清理结束后交给 onFailure。
    RCLCPP_WARN(logger(), "SEND_GOAL_TIMEOUT: waiting to finish goal cleanup for [%s]",
                action_name_.c_str());  // 每次请求记录一次，说明当前正在处理旧目标。
    client_instance_->cancel(request_);  // 持续处理本次请求，接受响应迟到时仍能取消。
  }

  auto feedback = std::move(request_->feedback);  // 移出当前批次，原容器交给后续回调。
  request_->feedback.clear();  // 显式保证下次 tick 从空容器开始。
  for(const auto& message : feedback)  // 保留原接口逐条调用 onFeedback 的行为。
  {
    if(request_->cancel_requested)
    {
      break;  // 已要求取消时停止调用业务反馈函数。
    }
    request_->feedback_status = onFeedback(message);  // 虚函数仅在 BT 线程调用。
    if(request_->feedback_status == NodeStatus::IDLE)
    {
      throw LogicError("onFeedback must not return IDLE");  // IDLE 只用于尚未开始的节点。
    }
    if(request_->feedback_status != NodeStatus::RUNNING)
    {
      client_instance_->cancel(request_);  // 业务反馈要求结束时，先取消本次目标。
    }
  }

  if(!request_->finished)
  {
    return NodeStatus::RUNNING;  // 等待正常结果，或等待取消确认后的最终结果。
  }
  if(request_->feedback_status != NodeStatus::RUNNING)
  {
    return check_status(request_->feedback_status);  // 保留 onFeedback 请求的结束状态。
  }
  if(request_->timed_out && request_->result.code != rclcpp_action::ResultCode::SUCCEEDED)
  {
    return check_status(onFailure(SEND_GOAL_TIMEOUT));  // 清理完成后才允许上层重试。
  }
  if(!request_->handle)
  {
    return check_status(onFailure(GOAL_REJECTED_BY_SERVER));  // 拒绝响应的句柄为空。
  }
  if(request_->result.code == rclcpp_action::ResultCode::ABORTED)
  {
    return check_status(onFailure(ACTION_ABORTED));  // 保留服务端中止的错误分类。
  }
  if(request_->result.code == rclcpp_action::ResultCode::CANCELED)
  {
    return check_status(onFailure(ACTION_CANCELLED));  // 保留外部取消的错误分类。
  }
  if(request_->result.code == rclcpp_action::ResultCode::UNKNOWN)
  {
    return check_status(onFailure(ACTION_CANCELLED));  // 已移除的句柄缺少结果时，跳过读取空结果消息。
  }
  return check_status(onResultReceived(request_->result));  // 已成功的旧目标按成功推进，避免重复导航。
}

template <class T>
inline void RosActionNode<T>::halt()
{
  const bool was_running = status() == NodeStatus::RUNNING;  // 保存停止前的节点状态。
  cancelGoal();  // 按请求状态取消，也覆盖尚未收到接受响应的目标。
  request_.reset();  // 父客户端接管待取消状态，BT 节点可以立即停止或销毁。
  if(was_running)
  {
    onHalt();  // 保留派生类的停止通知，在对象仍存活时调用。
  }
  resetStatus();  // 下次启动从 IDLE 开始，先等待旧请求清理。
}

template <class T>
inline void RosActionNode<T>::cancelGoal()
{
  std::lock_guard<std::mutex> lock(getMutex());  // 与 tick 和清理定时器使用同一把锁。
  if(client_instance_ && request_)
  {
    client_instance_->cancel(request_);  // 异步取消精确目标，结果仍由对应回调确认。
  }
}

}  // namespace BT
