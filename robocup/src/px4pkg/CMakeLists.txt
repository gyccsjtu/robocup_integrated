cmake_minimum_required(VERSION 3.0.2)
project(px4)

# 这是一个纯 shim 包：不编译任何代码，只是为了让 $(find px4) 能解析，
# 从而找到 scripts/gen_iris_sdf.sh、scripts/px4_delayed_start_multi.sh
# 以及转发到真正 SITL 二进制的 px4 可执行。

find_package(catkin REQUIRED)

catkin_package()
