# 自适应视觉伺服的机械臂自动抓取系统

在 KINOVA Gen3 lite 机械臂上，实现自适应视觉伺服的自动抓取系统。从自然语言指令得到抓取目标后， 借助 MediaPipe 实现目标物品的精确定位， 并自行设计生成抓取角度与路径，采用自研的自适应视觉伺服算法，将物品抓取到指定位置。

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
  
