# 3kg 双臂分拣机械臂（单臂）优化版选型说明

> 更新日期：2026-04-09  
> 本版目标：在低成本（飞特/OCS）路线下，把电机尺寸与自重纳入模型，固定 `L2 = 350 mm` 并优化 `L3`，重算扭矩与精度，并和 ALOHA 官方机械臂做定量对比。

## 1. 设计目标与本版结论

- 目标负载：末端抓取 `3 kg`
- 约束：`6DoF + 1 夹爪`，兼容 `ROS2 + VLA`
- 结构变化：`L2` 强制加长到 `350 mm`，`L3` 在“扭矩裕量 + 精度”之间优化
- 本版定案：`L3 = 250 mm`（可达半径 `~790 mm`），且在考虑电机自重后，关键关节仍有 `>=24%` 扭矩裕量

## 2. 几何、电机尺寸与质量模型

## 2.1 连杆参数（单臂）

- `L1 = 180 mm`
- `L2 = 350 mm`（固定）
- `L3 = 250 mm`（优化后）
- `L4 = 90 mm`
- `Ltool = 100 mm`
- 最大前伸半径：`R = L2 + L3 + L4 + Ltool = 790 mm`

## 2.2 电机选型（含尺寸/自重）

| 关节 | 型号 | 额定扭矩 | 外形尺寸 (mm) | 重量 | 机械减速 |
|---|---|---:|---|---:|---|
| J1 | OCS-D2001 | 60 kg.cm | 77 x 32 x 81.8 | 340 g | 1:1 |
| J2 | OCS-D9501 | 350 kg.cm | 120 x 60 x 111.5 | 1750 g | 2.5:1 |
| J3 | OCS-D6501 | 250 kg.cm | 100 x 50 x 97.4 | 896 g | 1.7:1 |
| J4 | OCS-D2002 | 85 kg.cm | 77 x 32 x 85.9 | 405 g | 1.6:1 |
| J5 | OCS-D2001 | 60 kg.cm | 77 x 32 x 81.8 | 340 g | 1:1 |
| J6 | FEETECH STS3250 | 16 kg.cm | 40 x 20 x 40.5 | 74.5 g | 1.5:1 |
| 夹爪 | FEETECH STS3215 | 10 kg.cm | 40.5 x 20 x 40.2 | 59 g | 指爪机构 |

说明：
- 扭矩换算：`1 kg.cm = 0.0980665 N.m`
- 关键变化：把原先较重的腕部电机替换为轻量组合（J6+夹爪），显著降低肩肘负担

## 3. L3 优化与扭矩复算（已计入电机自重）

## 3.1 计算假设

- 最不利工况：手臂近似水平前伸
- 负载：`3 kg`
- 动态系数：`1.35`（J2/J3），`1.25`（J4）
- 安全系数：`1.2`
- 电机、夹爪、连杆质量均计入力矩臂

## 3.2 L3 扫描结果

| L3 (m) | 前伸半径 (m) | J2设计扭矩 (N.m) | J3设计扭矩 (N.m) | J4设计扭矩 (N.m) | J2裕量 | J3裕量 | J4裕量 |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 0.24 | 0.78 | 60.91 | 27.56 | 9.56 | 40.9% | 51.2% | 39.5% |
| 0.25 | 0.79 | 61.63 | 28.28 | 9.56 | 39.2% | 47.4% | 39.5% |
| 0.26 | 0.80 | 62.35 | 29.00 | 9.56 | 37.6% | 43.7% | 39.5% |
| 0.27 | 0.81 | 63.07 | 29.72 | 9.56 | 36.1% | 40.2% | 39.5% |
| 0.28 | 0.82 | 63.79 | 30.44 | 9.56 | 34.5% | 36.9% | 39.5% |
| 0.30 | 0.84 | 65.22 | 31.87 | 9.56 | 31.6% | 30.8% | 39.5% |

优化结论：
- `L3 = 0.25 m` 是“加长但不过长”的折中点：臂展增益明确，同时扭矩裕量保持高位。

## 3.3 选定方案扭矩核验（L3=250 mm）

| 关节 | 设计需求扭矩 (N.m) | 电机额定+减速后可用扭矩 (N.m) | 裕量 |
|---|---:|---:|---:|
| J2 | 61.63 | 85.81 | 39.2% |
| J3 | 28.28 | 41.68 | 47.4% |
| J4 | 9.56 | 13.34 | 39.5% |

结论：在计入电机自重后，`J2/J3/J4` 理论上均满足 `3 kg` 目标并有可用裕量。

## 4. 精度复算（运动学传递 + 材料形变）

## 4.1 运动学传递误差（理论）

采用保守角误差模型（输出端）：
- J2：`0.2 deg / 2.5 = 0.08 deg`
- J3：`0.2 deg / 1.7 = 0.118 deg`
- J4：`0.2 deg / 1.6 = 0.125 deg`
- J5/J6：按 `0.2 deg` 估计

末端线性误差（线性化）：
- J2 贡献：`1.10 mm`
- J3 贡献：`0.91 mm`
- J4 贡献：`0.41 mm`
- J5/J6 合计：`0.35 mm` 量级
- 运动学 RSS：`1.54 mm`

## 4.2 连杆弹性形变误差（7075 管梁）

推荐截面：
- L2：`70 x 45 x 3 mm`
- L3：`60 x 40 x 3 mm`
- L4：`45 x 30 x 2.5 mm`

在当前工况下估算：
- L2 端挠度：`~0.056 mm`
- L3 端挠度：`~0.021 mm`
- L4 端挠度：`~0.003 mm`
- 梁模型总挠度：`~0.08 mm`

说明：材料形变不是主要误差源，主要误差来自关节角误差、传动间隙与装配误差。

## 4.3 理论重复定位精度（合成）

- 加入装配/轴承/法兰综合项（工程估计）：`~0.8 mm`
- 静态重复定位（RSS）：`~1.74 mm`
- 动态重复定位（RSS）：`~1.85 mm`
- 工程可实现区间（含控制与摩擦）：`~2-3 mm`

对分拣任务判断：
- 2-3 mm 级重复定位通常可满足传送带不堆叠积木分拣。

## 5. 与 ALOHA 官方机械臂对比（VX300s 基线）

ALOHA 常见 follower arm 使用 Interbotix VX300s（官方规格）：
- Reach：`750 mm`
- Working Payload：`750 g`
- Repeatability：`±1 mm`
- Accuracy：`±5 to ±8 mm`
- Total Servos：`9`（`1x gripper + 4x XM430 + 4x XM540`）

本方案（本 README 参数）对比：

| 维度 | ALOHA/VX300s 官方基线 | 本方案（OCS + FEETECH） |
|---|---|---|
| 最大前伸 | 750 mm | 790 mm |
| 额定负载目标 | 0.75 kg | 3 kg |
| 重复定位 | ±1 mm（官方） | 理论 1.7-1.9 mm，工程 2-3 mm |
| 绝对精度 | ±5 to ±8 mm（官方） | 预计 4-7 mm（需手眼标定） |
| 驱动结构 | 9 舵机（含肩肘双机并联） | 7 执行单元 + 机械减速 |
| 适配难度 | 生态成熟 | 需做 RS485/Modbus 硬件接口适配 |

补充（成本感知）：
- ALOHA 官方链路中常见电机（XM430/XM540）零售价较高，且肩肘常靠多电机并联；
- 本方案把高扭矩放在 OCS 近端，远端轻量化，目标是以更低 BOM 达到更高负载；
- 但为了达到可比精度，需要额外做好减速预紧、装配精度与控制补偿。

## 6. ROS2 与 VLA 接口保持不变

- 单臂动作接口仍为 7 维：`[j1, j2, j3, j4, j5, j6, gripper]`
- 控制链保持：`ros2_control + joint_trajectory_controller`
- 需要更新：`URDF` 惯量参数、`joint_limits.yaml`（按本版扭矩和减速比回填）

## 7. 落地建议

- 先做单臂台架验证（J2/J3/J4/J6+夹爪），实测：
  - 温升和连续运行占空比
  - 3kg 工况轨迹跟踪误差
  - 重复抓取成功率
- 若实测重复定位 >3 mm，优先改进顺序：
  1. 提高 J2/J3 关节侧编码器与标定质量
  2. 提高同步带预紧与轴承预紧
  3. 再考虑提高减速比或升级 J6 执行器

## 8. 参考链接（本次复算使用）

### OCS / OCSERVO
- OCS-D9501（EN）：<https://en.ocservo.com/?page_id=17622&post_type=products>
- OCS-D6501（CN）：<https://www.ocservo.com/?page_id=17432&post_type=products>
- OCS-D2002（CN）：<https://www.ocservo.com/?page_id=17396&post_type=products>
- OCS-D2001（CN）：<https://www.ocservo.com/?page_id=17294&post_type=products>

### FEETECH
- STS3250：<https://www.feetechrc.com/en/562636.html>
- STS3215：<https://www.feetechrc.com/en/525603.html>
- SM-260B-C001：<https://www.feetechrc.com/en/573021.html>
- SM120BL（Modbus RTU）：<https://www.feetechrc.com/24v-120kgcm-modbus-rtu%E8%88%B5%E6%9C%BA.html>

### ALOHA / VX300s 与 DYNAMIXEL
- VX300s 官方规格：<https://docs.trossenrobotics.com/interbotix_xsarms_docs/specifications/vx300s.html>
- XM430-W350-R：<https://www.robotis.us/dynamixel-xm430-w350-r/>
- XM540-W270-R：<https://www.robotis.us/dynamixel-xm540-w270-r/>

### ROS2 / VLA
- ros2_control hardware interface：<https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/hardware_interface_types_userdoc.html>
- OpenVLA：<https://github.com/openvla/openvla>
- LeRobot action representations：<https://huggingface.co/docs/lerobot/main/action_representations>
