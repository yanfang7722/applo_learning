# cmake 最低版本需求
cmake_minimum_required(VERSION 3.13)

# 工程名称
project(Dijkstra)

# 设置
set(CMAKE_CXX_STANDARD 11)
#set (CMAKE_CXX_FLAGS  -g)
add_compile_options(-g)

# 编译源码生成目标
add_executable(Dijkstra dijkstra.cpp )

#matplot设置
find_package(Python2 COMPONENTS Development NumPy)
target_include_directories(Dijkstra PRIVATE
    /usr/include/python2.7 
    /usr/lib/python2.7/dist-packages/numpy/core/include
)
target_link_libraries(Dijkstra Python2::Python Python2::NumPy)



