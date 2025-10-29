# 说明

此工程为ros下图像压缩传输解压缩，采用H.264编解码，用以减少图像话题所占带宽

# 环境准备

```bash
sudo apt-get install libx264-dev 
```

# 图像话题编码
```bash
source devel/setup.bash
roslaunch ros_h264_streamer run_encode.launch
```
|参数名称|功能描述|备注|
|---|---|---|
|image_topic|接收相机image的topic话题| - |
|encoded_topic|发布压缩后图像数据话题| - |
|q_level|压缩率设置| 取值范围1-100，越大压缩效果越强 |
|fps|图像话题帧率| - |

# 图像话题解码
```bash
source devel/setup.bash
roslaunch ros_h264_streamer run_decode.launch
```
|参数名称|功能描述|备注|
|---|---|---|
|image_restored_topic|发布解压缩的相机image的topic话题| - |
|encoded_topic|接收待解压缩图像数据| - |

# 致谢
项目源码改编来自 https://github.com/gergondet/ros_h264_streamer 工程