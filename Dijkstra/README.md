#  Dijkstra


## 参数说明
文件开始全局变量定义
地图大小、分辨率
 ```
double xmin = -10.0;
double ymin = -10.0;
double xmax = 60.0;
double ymax = 60.0;
double grid_size = 2.0;
 ```
 机器人大小
 ```
 double robot_size = 2.0;
 ```
地图路径
```
string file_dir = "//media/yangf/document/test_c++/Dijkstra/obs_map.csv";
```
文件打开失败时输出“open file failed !!”，并退出程序。
计算结果文件保存与该路径,包括：
obs_map_result.csv -存储最终搜索的路径
result.png - 搜索结果图片

## 显示说明
```
const bool PROCESS_PLOT = true;
```
该变量用于控制是否显示中间搜索过程，默认打开

## matplotcpp使用说明
工程中matplotcpp适配Python2
```
#include </usr/include/python2.7/Python.h>
#include </usr/lib/python2.7/dist-packages/numpy/core/include/numpy/arrayobject.h>
```
使用时需修改对应Python2 和numpy库的包含路径

