# Inspire Hand

本项目旨在基于Modbus TCP接口，完成因时四代灵巧手运动控制、触觉信息采集及数据回放的开发。

希望能帮助你完成因时四代灵巧手 RH56DFTP_0RL 和 RH56DFTP_2R 运动控制开发,目前触觉采集频率大约维持在30Hz。

## Conda 环境配置
```bash
conda create -n rh_hand python=3.9 
conda activate rh_hand 
cd Inspire_hand
pip install -r requirements.txt
```

## 功能大致介绍
### 1、控制
参考`normal()`函数进行修改即可，具体参考`rh_hand_RH56DFTP_2R.py`
### 2、触觉信息采集
参考`read_multiple_registers_map(duration_sec=0, target_hz=120, gui_fps=30, ave_png_every=0)`，具体参考`rh_hand_RH56DFTP_2R.py`
### 3、触觉信息回放
参考`replay_log("/home/galaxy/Inspire_hand/logs/tactile_test.jsonl", fps=8,speed=2.0，loop=False)`，注意这里地址是绝对路径，运行前根据实际情况修改绝对路径，具体参考`rh_hand_RH56DFTP_0RL.py`。/log文件夹下有离线读取触觉信息的示例，通过读取示例信息判断代码是否能够跑通。

## 备注
由于因时官方每一批的灵巧手的固件存在不同，主要表现在触觉感知可能不同批次的在按压后反馈的按压位置不同。如果出现上述现象，请检查并调整`format_finger_data()`和`format_palm_data()`函数，可能实际的硬件对应读取到的触觉数据需要做相应的转置等矩阵变换。
