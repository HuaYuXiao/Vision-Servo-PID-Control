# Visual-servo-NLP-based-6DOF-Manipulator-Grasp-System
SUSTech EE368

## Environment
- **platform**: Linux
- **language**: Python

## Division of work
- Hongjing Tang: visual-servo control and camera configuration
- Yuxiao Hua: multi-thread, NLP and final report
- Xizhe Hao: voice control and PPT presentation

## How to run the code
Download all files under the "code" folder, and then run the main.py file.

Note: Before running, make sure to modify the reference files to the appropriate location on your computer.
![image](https://github.com/HuaYuXiao/Visual-servo-NLP-based-6DOF-Manipulator-Grasp-System/assets/100033111/668a3de3-0216-46da-9722-6250f476714b)

## 历史版本

### 2023年9月3日

实现了文字输入NLP功能，基于OpenAI与Google翻译两个接口，具体代码详见NLP.py。

```shell
D:\software\anaconda3\python.exe D:\iCloudDrive\项目\NLP\代码\main.py 
请告诉我你的需求：我喜欢冰箱和苹果，我讨厌山和香蕉，他超爱梨和鸭子。
喜欢的物品：冰箱、苹果、梨、鸭子。
['refrigerator', 'apple', 'pear', 'Duck']

Process finished with exit code 0
```
