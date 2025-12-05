cmake_minimum_required(VERSION 3.8)
project(ramen_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# ✅ 메시지 파일
set(msg_files
  "msg/RamenOrder.msg"
  "msg/CookedRamen.msg"
)

# ✅ 서비스 파일 (OrderService 추가!)
set(srv_files
  "srv/OrderService.srv"
)

# ✅ 액션 파일
set(action_files
  "action/ServeRamen.action"
  "action/RequestRemake.action"
)

# ✅ 인터페이스 생성
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES builtin_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()