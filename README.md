# 功能介绍

**1.racing_traffic_light_detection**
使用opencv解析HSV色彩空间实现红绿灯的检测

**2.racing_image_color_analysis**
输出图片中指定像素的HSV三通道的值

# 使用方法

## 准备工作

具备真实的机器人或机器人仿真模块，包含相机及RDK套件，并且能够正常运行。

## 安装功能包

**1.安装功能包**

启动机器人后，通过终端SSH或者VNC连接机器人，点击本页面右上方的“一键部署”按钮，复制如下命令在RDK的系统上运行，完成相关Node的安装。

```bash
sudo apt update
sudo apt install -y tros-racing-light-detection-cv
```

**2.运行红绿灯检测功能**

```shell
source /opt/tros/local_setup.bash
ros2 run racing_light_detection_cv racing_traffic_light_detection
```

**3.运行图片色彩空间解析功能**

```shell
source /opt/tros/local_setup.bash
ros2 run racing_light_detection_cv racing_image_color_analysis
```


# 接口说明

## 话题

### 红绿灯检测功能Pub话题

| 名称                          | 消息类型                                                     | 说明                                                   |
| ----------------------------- | ------------------------------------------------------------ | ------------------------------------------------------ |
| /process_image/mask_red                     | sensor_msgs/msg/Image                | 红色物体的掩膜                 |
| /process_image/mask_green                    | sensor_msgs/msg/Image               | 绿色物体的掩膜                 |
| /process_image                   | sensor_msgs/msg/Image                   | 图片与识别结果的结合图                 |
| /traffic_light_detection                 | ai_msgs/msg/Point                | 红绿灯的类型以及位置                 |

### 红绿灯检测功能Sub话题
| 名称                          | 消息类型                                                     | 说明                                                   |
| ----------------------------- | ------------------------------------------------------------ | ------------------------------------------------------ |
| /image_raw                    | sensor_msgs/msg/Image                                    | 接收相机发布的图片消息的名称               |


### 图片解析功能Sub话题

| 名称                          | 消息类型                                                     | 说明                                                   |
| ----------------------------- | ------------------------------------------------------------ | ------------------------------------------------------ |
| /image_raw                    | sensor_msgs/msg/Image                                    | 接收相机发布的图片消息的名称               |


## 参数

### 红绿灯检测功能可配置参数

| 参数名                | 类型        | 说明   |
| --------------------- | ----------- | -------------------------------------------------------------------------------------------------- |
| sub_img_topic       | string |接收相机发布的图片消息的名称，默认值为/image_raw  |
| pub_mask       | bool |  是否发布红色和绿色物体的掩膜图片，默认值为False |
| pub_result       | bool |  是否发布图片与识别结果的结合图，默认值为False |
| size_threshold       | int |  识别到灯的大小的阈值，默认值为2000 |
| lower_red_array       | array |  红灯分割HSV三通道的最小值，默认值为[150, 70, 250] |
| upper_red_array       | array |  红灯分割HSV三通道的最大值，默认值为[200, 160, 255] |
| lower_green_array       | array |  绿灯分割HSV三通道的最小值，默认值为[35, 200, 200] |
| upper_green_array       | array |  绿灯分割HSV三通道的最大值，默认值为[100, 255, 255] |

### 图片解析功能可配置参数

| 参数名                | 类型        | 说明   |
| --------------------- | ----------- | -------------------------------------------------------------------------------------------------- |
| sub_img_topic       | string |接收相机发布的图片消息的名称，默认值为/image_raw  |