# Vision Servo PID Control for Robotic Arm

![HitCount](https://img.shields.io/endpoint?url=https%3A%2F%2Fhits.dwyl.com%2FHuaYuXiao%2FAdaptive-Vision-Servo-for-Robotic-Arm-Automatic-Gripping-System.json%3Fcolor%3Dpink)
![Static Badge](https://img.shields.io/badge/ROS-kinetic-22314E?logo=ros)
![Static Badge](https://img.shields.io/badge/Ubuntu-16.04-E95420?logo=ubuntu)
![Static Badge](https://img.shields.io/badge/Python-_-3776AB?logo=python)
![Static Badge](https://img.shields.io/badge/TensorFlow-_-FF6F00?logo=tensorflow)
![Static Badge](https://img.shields.io/badge/OpenAI-_-412991?logo=openai)
![Static Badge](https://img.shields.io/badge/Google_Translate-_-4285F4?logo=googletranslate)


将自然语言指令处理后确定抓取目标， 借助 D435i 深度相机和 MobileNetV3 算法实现目标的精确定位，采用视觉伺服 PID 控制算法， 控制 KINOVA Gen3 lite 机械臂将目标抓取到指定位置。


## Video

View on YouTube: https://youtu.be/GrROyGOzRIE


## MobileNetV3

MobileNetV3 是 Google 在移动端设备上推出的第三代轻量级卷积神经网络（MobileNet）。它的设计目标是在保持高度准确性的同时，将模型的计算和参数量尽可能减小，以适应移动设备上的实时应用和边缘计算场景。

MobileNetV3 的一些特点包括：

1. **网络架构优化：** MobileNetV3 通过引入一系列网络架构的优化，包括倒残差块（Inverted Residuals）、线性瓶颈和注意力机制，以提高网络的表达能力和性能。

2. **轻量级设计：** MobileNetV3 采用轻量级设计，通过深度可分离卷积和通道注意力等技术，显著减小了网络的计算量和参数数量。这使得 MobileNetV3 在移动设备上运行更加高效。

3. **模型速度和准确性平衡：** MobileNetV3 的设计注重在保持高准确性的同时，加强对速度的优化。它通过网络架构的调整，取得了在速度和准确性之间的平衡，适合实时应用场景。

4. **网络定制能力：** MobileNetV3 提供了多个版本的模型，包括 MobileNetV3-Large 和 MobileNetV3-Small，以满足不同应用场景和硬件资源的需求。

5. **网络裁剪：** MobileNetV3 还支持网络裁剪，用户可以根据具体需求选择裁剪掉不需要的部分，以满足特定的应用场景和硬件资源。

总体而言，MobileNetV3 是一种面向移动设备和嵌入式系统的高效神经网络架构，通过多方面的优化提供了在轻量级模型中较高的性能。


## 如何运行

Download all files under the `code` folder, and then run the `main.py` file.

Note: Before running, make sure to modify the reference files to the appropriate location on your computer.

```python
classFile = '/home/ee368-7/Desktop/EE368_lab/project/Resources/coco.names'
configPath = '/home/ee368-7/Desktop/EE368_lab/project/Resources/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt'
weightsPath = '/home/ee368-7/Desktop/EE368_lab/project/Resources/frozen_inference_graph.pb'
```


## 历史版本

### 2023年9月3日

新增了以下内容：首先通过实时语音输入或文字输入，然后基于[OpenAI](https://github.com/chatanywhere/GPT_API_free)与Google翻译两个接口，实现NLP语义理解，最后输出目标物品的名字。

具体代码详见[NLP](https://github.com/HuaYuXiao/Visual-servo-NLP-based-6DOF-Manipulator-Grasp-System/tree/main/code/NLP)文件夹下的两个程序。

```bash
D:\software\anaconda3\python.exe D:\iCloudDrive\项目\NLP\代码\main.py 
请告诉我你的需求：我喜欢冰箱和苹果，我讨厌山和香蕉，他超爱梨和鸭子。
喜欢的物品：冰箱、苹果、梨、鸭子。
['refrigerator', 'apple', 'pear', 'Duck']

Process finished with exit code 0
```

## TODO

- [x] 实时语音输入或文字输入
- [x] NLP语义理解输出目标物品
- [ ] 确定最佳抓取角度


## Contributions

- Hongjing Tang: visual-servo control, camera configuration
- Yuxiao Hua: multi-thread, NLP and final report
- Xizhe Hao: voice control and PPT presentation
