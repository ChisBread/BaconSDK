cmake_minimum_required(VERSION 3.10)
project(bacon_project)

# 设置C++标准
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(SOURCES
    bacon.cc
    common.cc
    platform.cc
)
# 添加源文件
add_executable(bacon_cli bacon_cli.cc)
target_link_libraries(bacon_cli PUBLIC libbacon)

add_library(libbacon SHARED ${SOURCES})
add_library(libbacon_static STATIC ${SOURCES})
set_target_properties(libbacon PROPERTIES OUTPUT_NAME "bacon")
set_target_properties(libbacon_static PROPERTIES OUTPUT_NAME "bacon")
install(TARGETS bacon_cli libbacon DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)

# 添加其他头文件目录（如果需要）
# include_directories(/path/to/headers)

# 如果有额外的编译选项，可以在这里添加
# target_compile_options(bacon_cli PRIVATE -O3 -ltcmalloc_minimal -static)
target_compile_options(libbacon PRIVATE -O3 -ltcmalloc_minimal -static)