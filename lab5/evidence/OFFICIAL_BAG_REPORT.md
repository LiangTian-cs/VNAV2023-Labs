# Lab5 官方数据与课程 ROS 包验收

日期：2026-10-23。代码 versions/005。没有修改原 003/004、共享环境或 Lab3/4。
两份指定文件由用户桌面上传，两端 SHA256 一致；不冒充发布方签名校验。

## 运行范围

课程 lab_5 包实际 catkin 构建，保留课程原类接口和两个 launch 名称。新 headless course_entry.cpp 替代未实现的原入口；原入口仅作参考保留，不编译。
ROS master 使用独立端口 11555-11559。视频 0.2 倍速回放，非实时性能测试；不启动 Unity 或控制器，只回放图像与时钟。

## 官方 bag 全序列结果

| 数据 | 算法 | 处理帧/原帧 | 配对数 | 平均匹配数 | 平均内点数 | 平均内点率 | 验收 |
|---|---|---:|---:|---:|---:|---:|---|
| RealSense | SIFT | 1584/1584 | 1583 | 258.41 | 249.03 | 95.73% | PASS |
| RealSense | LK | 1584/1584 | 1583 | 58.45 | 58.44 | 99.98% | PASS |
| TESSE smooth | SIFT | 1386/1386 | 1385 | 92.95 | 80.57 | 86.06% | PASS |
| TESSE smooth | LK | 1386/1386 | 1385 | 40.19 | 38.83 | 94.70% | PASS |

匹配数：SIFT 为 FLANN 2-NN 后 0.8 ratio 筛选；LK 为通过状态、边界及 <=1 px 前后向检查的跟踪点。
内点采用课程 F-RANSAC，阈值 3 px、置信度 0.99。平均内点率是逐帧比率均值，不是总数比率，更不是人工真值准确率。

## 双图回归

课程 box/box_in_scene：604/969 个 SIFT 关键点，128 维描述子；604 原始匹配，101 筛选后匹配，79 内点，内点率 78.2178%。

## 方法及限制

SIFT 每对相邻帧重新检测/匹配，不等同维护持续特征 ID。LK 使用 Harris、最多 500 点、窗口 21x21、3 层金字塔，少于 50 点时重检测。
LK 依赖近似亮度恒定、局部运动一致和较小帧间位移；遮挡、模糊和低纹理会失效。慢速回放不改变原始相邻图像位移，只避免计算过慢造成队列丢帧。

## 证据

各运行子目录包含 events.csv、逐帧指标 CSV、SUMMARY.json、VERIFICATION.json、launch.log、command.json、statistics.png。
SIFT PNG 是首对及每 100 对的快照，同名更新，保留最近一次采样；LK 保留第 1 对和每 30 对 PNG。所有配对均有 CSV，不能把采样图当成逐帧录像。
INPUTS.json 保存 bag 大小和 SHA256；SOURCE_SHA256.json 保存源码校验。

## 复现

```bash
cd /root/autodl-tmp/vnav2023
source scripts/activate_lab5.sh
cd lab5/versions/005/ws
catkin_make -DPYTHON_EXECUTABLE="$VNAV_ENV/bin/python" -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DCMAKE_BUILD_TYPE=Release -j4
source devel/setup.bash
cd ..
# 重跑前将 run_official.py 的 OUT 设置为新的结果目录，默认拒绝覆盖旧结果。
python run_official.py
python summarize_official.py
```

## 数值异常与验收边界

004 的 TESSE LK 因 OpenCV4.13 基础矩阵估计断言拒绝 8 帧，失败证据保留在 results/official-004-r2/1_lk。
005 将异常标为 geometry_valid=0，几何内点计零，保留图像及合法 LK 跟踪，不换估计器或降低阈值。F_FAILURES.log 和对应点 CSV 保留完整证据。
reproduce_f_failure.py 使用 11/12 个有限且互异的点，在不运行 ROS 的情况下复现相同断言；修复的是异常处理边界，不宣称修好了 OpenCV 内部算法。
TESSE LK 曾因 rosbag 读取终端输入而暂停，使用其专用 ROS service 恢复；无跳帧/重置。脚本现将子进程标准输入接到 DEVNULL，防止复现时键盘干扰。
本轮按全序列帧数、逐一时间戳及相邻配对数验收；没有以很高的平均内点率替代完整性检查。

## 提交边界

必做实验官方数据、课程包集成、两 bag 全序列运行证据已补齐，以 VERIFICATION.json 为准。理论在 theory/D1_D2_DERIVATIONS.md 和 NUMERICAL_CHECK.json。
最终 PDF 排版及本校提交格式仍待整理，本文 Markdown 不冒充正式 PDF。
选做 ORB/FAST+BRIEF/Farneback 沿用已有 Python 补充结果，不宣称全部 C++ 类已实现；SURF nonfree 不可用而跳过，没有重编环境。
