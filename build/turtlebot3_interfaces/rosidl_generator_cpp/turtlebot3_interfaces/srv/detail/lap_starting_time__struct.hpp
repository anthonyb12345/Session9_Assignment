// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from turtlebot3_interfaces:srv/LapStartingTime.idl
// generated code does not contain a copyright notice

#ifndef TURTLEBOT3_INTERFACES__SRV__DETAIL__LAP_STARTING_TIME__STRUCT_HPP_
#define TURTLEBOT3_INTERFACES__SRV__DETAIL__LAP_STARTING_TIME__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__turtlebot3_interfaces__srv__LapStartingTime_Request __attribute__((deprecated))
#else
# define DEPRECATED__turtlebot3_interfaces__srv__LapStartingTime_Request __declspec(deprecated)
#endif

namespace turtlebot3_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct LapStartingTime_Request_
{
  using Type = LapStartingTime_Request_<ContainerAllocator>;

  explicit LapStartingTime_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit LapStartingTime_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    turtlebot3_interfaces::srv::LapStartingTime_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const turtlebot3_interfaces::srv::LapStartingTime_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<turtlebot3_interfaces::srv::LapStartingTime_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<turtlebot3_interfaces::srv::LapStartingTime_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      turtlebot3_interfaces::srv::LapStartingTime_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<turtlebot3_interfaces::srv::LapStartingTime_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      turtlebot3_interfaces::srv::LapStartingTime_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<turtlebot3_interfaces::srv::LapStartingTime_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<turtlebot3_interfaces::srv::LapStartingTime_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<turtlebot3_interfaces::srv::LapStartingTime_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__turtlebot3_interfaces__srv__LapStartingTime_Request
    std::shared_ptr<turtlebot3_interfaces::srv::LapStartingTime_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__turtlebot3_interfaces__srv__LapStartingTime_Request
    std::shared_ptr<turtlebot3_interfaces::srv::LapStartingTime_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LapStartingTime_Request_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const LapStartingTime_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LapStartingTime_Request_

// alias to use template instance with default allocator
using LapStartingTime_Request =
  turtlebot3_interfaces::srv::LapStartingTime_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace turtlebot3_interfaces


#ifndef _WIN32
# define DEPRECATED__turtlebot3_interfaces__srv__LapStartingTime_Response __attribute__((deprecated))
#else
# define DEPRECATED__turtlebot3_interfaces__srv__LapStartingTime_Response __declspec(deprecated)
#endif

namespace turtlebot3_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct LapStartingTime_Response_
{
  using Type = LapStartingTime_Response_<ContainerAllocator>;

  explicit LapStartingTime_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit LapStartingTime_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    turtlebot3_interfaces::srv::LapStartingTime_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const turtlebot3_interfaces::srv::LapStartingTime_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<turtlebot3_interfaces::srv::LapStartingTime_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<turtlebot3_interfaces::srv::LapStartingTime_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      turtlebot3_interfaces::srv::LapStartingTime_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<turtlebot3_interfaces::srv::LapStartingTime_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      turtlebot3_interfaces::srv::LapStartingTime_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<turtlebot3_interfaces::srv::LapStartingTime_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<turtlebot3_interfaces::srv::LapStartingTime_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<turtlebot3_interfaces::srv::LapStartingTime_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__turtlebot3_interfaces__srv__LapStartingTime_Response
    std::shared_ptr<turtlebot3_interfaces::srv::LapStartingTime_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__turtlebot3_interfaces__srv__LapStartingTime_Response
    std::shared_ptr<turtlebot3_interfaces::srv::LapStartingTime_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LapStartingTime_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const LapStartingTime_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LapStartingTime_Response_

// alias to use template instance with default allocator
using LapStartingTime_Response =
  turtlebot3_interfaces::srv::LapStartingTime_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace turtlebot3_interfaces

namespace turtlebot3_interfaces
{

namespace srv
{

struct LapStartingTime
{
  using Request = turtlebot3_interfaces::srv::LapStartingTime_Request;
  using Response = turtlebot3_interfaces::srv::LapStartingTime_Response;
};

}  // namespace srv

}  // namespace turtlebot3_interfaces

#endif  // TURTLEBOT3_INTERFACES__SRV__DETAIL__LAP_STARTING_TIME__STRUCT_HPP_
