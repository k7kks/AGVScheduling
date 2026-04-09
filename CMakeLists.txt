cmake_minimum_required(VERSION 3.10)
project(agv_cluster_scheduling LANGUAGES CXX)  # 明确指定语言为CXX

# 设置 C++ 标准为 C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)  # 禁用编译器扩展，确保严格遵循 C++17 标准


# 生成 compile_commands.json（便于 IDE/静态分析）
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


# 设置可执行文件输出目录（统一到build/bin，避免build根目录杂乱）
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# 优先使用系统安装的 rabbitmq-c；若不存在，则回退到仓库内的本地依赖目录。
set(RABBITMQ_LOCAL_PREFIX "${CMAKE_SOURCE_DIR}/.deps/local-rabbitmq")
find_path(RABBITMQ_INCLUDE_DIR NAMES amqp.h)
find_library(RABBITMQ_LIB NAMES rabbitmq)
if(NOT RABBITMQ_INCLUDE_DIR)
    find_path(RABBITMQ_INCLUDE_DIR
        NAMES amqp.h
        PATHS "${RABBITMQ_LOCAL_PREFIX}/include"
    )
endif()
if(NOT RABBITMQ_LIB)
    find_library(RABBITMQ_LIB
        NAMES rabbitmq
        PATHS "${RABBITMQ_LOCAL_PREFIX}/lib"
    )
endif()

# 头文件搜索路径（保持现有路径，确保所有include都能被找到）
include_directories(
    include
    include/common
    include/algorithm
    include/algorithm/base
    include/data
    include/lib
    include/parser
    include/message
    include/path_planning
    ${RABBITMQ_INCLUDE_DIR}
)

# 公共源码分组：任务分配核心逻辑
set(ALGO_CORE_SOURCES
    src/algorithm/PostaTaskAllocator.cpp
    src/algorithm/GreedyTaskAllocator.cpp
    src/algorithm/base/TaskAllocationUtils.cpp
    src/algorithm/base/TaskAllocatorBase.cpp
    src/algorithm/CostMatrixGenerator.cpp
    src/algorithm/ShortestPathUpdater.cpp
    src/common/AllocationReachability.cpp
    src/common/TaskReachabilityFilter.cpp
    src/common/CongestionRegionUtils.cpp
    src/data/MapInfo.cpp
    src/parser/JsonParser.cpp
)

# 公共源码分组：全局路径规划模块
set(PATH_PLANNING_SOURCES
    src/path_planning/StaticPathTable.cpp
    src/path_planning/GlobalPathPlanner.cpp
    src/path_planning/AStarPathFinder.cpp
    src/path_planning/RobustPathPlanner.cpp
    src/path_planning/PriorityPathPlanner.cpp
    src/path_planning/PathPlanningHelper.cpp
    src/common/NodeReservationTable.cpp
    src/common/AmrPositionResolver.cpp
)

# 链接依赖库（明确PRIVATE链接方式，符合现代CMake规范）
find_package(Threads REQUIRED)

# 链接 RabbitMQ 相关库（使用 rabbitmq-c）
if(NOT RABBITMQ_LIB OR NOT RABBITMQ_INCLUDE_DIR)
    message(FATAL_ERROR "rabbitmq-c 未找到，请先安装系统包或准备 .deps/local-rabbitmq")
endif()

# 便捷目标：清理 CMake 缓存（遇到“Cache源路径不一致”时可用）
add_custom_target(distclean
    COMMAND ${CMAKE_COMMAND} -E rm -f ${CMAKE_BINARY_DIR}/CMakeCache.txt
    COMMAND ${CMAKE_COMMAND} -E rm -rf ${CMAKE_BINARY_DIR}/CMakeFiles
    COMMAND ${CMAKE_COMMAND} -E rm -rf ${CMAKE_BINARY_DIR}/bin
    COMMENT "Remove CMake cache and generated files under build/."
)

# 便捷目标：重新配置（在执行 distclean 后调用）
add_custom_target(reconfigure
    COMMAND ${CMAKE_COMMAND} -S ${CMAKE_SOURCE_DIR} -B ${CMAKE_BINARY_DIR}
    COMMENT "Re-run CMake configuration in current build directory."
)

# --- Tools: external_receiver (listen external messages only, print info) ---
add_executable(external_receiver
    src/tools/external_receiver.cpp
    ${ALGO_CORE_SOURCES}
    src/message/ResultPublisher.cpp
    src/message/AlgoPublisher.cpp
    src/message/TrafficPathPublisher.cpp
    ${PATH_PLANNING_SOURCES}        # 结果发布内部集成路径规划
)
target_link_libraries(external_receiver PRIVATE
    Threads::Threads m ${RABBITMQ_LIB}
)

# --- Tools: external senders (business message simulators) ---
add_executable(external_task_sender
    src/tools/external_task_sender.cpp
)
target_link_libraries(external_task_sender PRIVATE
    Threads::Threads m ${RABBITMQ_LIB}
)

add_executable(external_status_sender
    src/tools/external_status_sender.cpp
)
target_link_libraries(external_status_sender PRIVATE
    Threads::Threads m ${RABBITMQ_LIB}
)

add_executable(external_config_sender
    src/tools/external_config_sender.cpp
)
target_link_libraries(external_config_sender PRIVATE
    Threads::Threads m ${RABBITMQ_LIB}
)

add_executable(external_map_sender
    src/tools/external_map_sender.cpp
)
target_link_libraries(external_map_sender PRIVATE
    Threads::Threads m ${RABBITMQ_LIB}
)

add_executable(external_path_request_sender
    src/tools/external_path_request_sender.cpp
)
target_link_libraries(external_path_request_sender PRIVATE
    Threads::Threads m ${RABBITMQ_LIB}
)

add_executable(external_trail_request_sender
    src/tools/external_trail_request_sender.cpp
)
target_link_libraries(external_trail_request_sender PRIVATE
    Threads::Threads m ${RABBITMQ_LIB}
)
