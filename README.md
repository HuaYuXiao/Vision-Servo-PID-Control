# 视觉伺服的NLP实时语音控制的6DOF机械臂抓取系统

SUSTech EE368 期末设计

该项目是一个基于ROS的6DOF机械臂抓取系统。通过NLP实时语音控制得到意向物品，借助MediaPipe计算机视觉算法实现目标物品的识别，并通过视觉伺服误差校正实现准确抓取。

## 如何运行

Download all files under the "code" folder, and then run the main.py file.

Note: Before running, make sure to modify the reference files to the appropriate location on your computer.

![image](https://github.com/HuaYuXiao/Visual-servo-NLP-based-6DOF-Manipulator-Grasp-System/assets/100033111/668a3de3-0216-46da-9722-6250f476714b)

## 历史版本

### 2023年9月3日

新增了以下内容：首先通过实时语音输入或文字输入，然后基于[OpenAI](https://github.com/chatanywhere/GPT_API_free)与Google翻译两个接口，实现NLP语义理解，最后输出目标物品的名字。

具体代码详见[NLP](https://github.com/HuaYuXiao/Visual-servo-NLP-based-6DOF-Manipulator-Grasp-System/tree/main/code/NLP)文件夹下的两个程序。

```shell
D:\software\anaconda3\python.exe D:\iCloudDrive\项目\NLP\代码\main.py 
请告诉我你的需求：我喜欢冰箱和苹果，我讨厌山和香蕉，他超爱梨和鸭子。
喜欢的物品：冰箱、苹果、梨、鸭子。
['refrigerator', 'apple', 'pear', 'Duck']

Process finished with exit code 0
```

## 小组分工

- Hongjing Tang: visual-servo control and camera configuration
  
- Yuxiao Hua: multi-thread, NLP and final report
  
- Xizhe Hao: voice control and PPT presentation
  
