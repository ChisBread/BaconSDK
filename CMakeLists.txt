cmake_minimum_required(VERSION 3.10)
project(bacon_project)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(SOURCES
    bacon.cc
    common.cc
    platform.cc
)

add_executable(bacon_cli bacon_cli.cc)
target_link_libraries(bacon_cli PUBLIC libbacon_static)

add_library(libbacon SHARED ${SOURCES})
target_link_libraries(libbacon  -static-libgcc;-static-libstdc++)
set_target_properties(libbacon PROPERTIES OUTPUT_NAME "bacon")

add_library(libbacon_static STATIC ${SOURCES})
set_target_properties(libbacon_static PROPERTIES OUTPUT_NAME "bacon")
target_link_libraries(libbacon_static  -static-libgcc;-static-libstdc++)

install(TARGETS bacon_cli DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
install(TARGETS libbacon libbacon_static DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)



#TODO auto detect tc_malloc

target_compile_options(libbacon PRIVATE -O3)
target_compile_options(libbacon_static PRIVATE -O3)