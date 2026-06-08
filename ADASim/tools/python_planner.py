#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file algo_node.py
@brief 自动驾驶算法仿真平台 (ADASim) - 外部规控大脑节点
@date 2026-04

@details
本节点作为独立的算法进程，通过 TCP Socket 与 C++ 仿真底座进行全双工通信。
* 【算法栈架构】
1. Mode 0 - Lattice Planner: 基于高斯风险场与多目标代价函数 (车道保持、平滑度、防碰撞) 的横向采样寻优。
2. Mode 1 - APF (Artificial Potential Field): 模拟斥力场的人工势场法避障。
3. Mode 2 - MPC + PID 联合控制: 结合车辆运动学模型 (Bicycle Model) 的前馈模型预测控制，与 PID 误差反馈控制。
"""

import socket
import json
import time
import math
import logging
from typing import List, Dict, Any

# =============================================================================
# [系统配置与日志初始化]
# =============================================================================
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s.%(msecs)03d [%(levelname)s] %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger("ADASim_Brain")

# =============================================================================
# [算法超参数配置区 (Hyperparameters)]
# =============================================================================

# --- 1. Lattice 规划器参数 ---
CANDIDATE_OFFSETS = [-2, -1, 0, 1, 2]  # 候选横向采样通道 (对应相邻车道)

W_LANE = 40.0        # 代价权重：惩罚偏离中心车道的行为
W_SMOOTH = 10.0      # 代价权重：惩罚方向盘的剧烈晃动 (追求体感平滑)
W_COLLISION = 1000.0 # 代价权重：一票否决级的避障安全权重

# 高斯风险场 (Gaussian Risk Field) 扩散系数
SIGMA_X = 8.0        # 纵向风险衰减速度 (车辆前后预留的安全空间较大)
SIGMA_Y = 1.2        # 横向风险衰减速度 (侧向允许贴得较近)

# 车辆物理碰撞包络 (米)
VEHICLE_LENGTH = 4.6  
VEHICLE_WIDTH = 2.2   

# --- 2. 运动学底层控制器参数 (MPC & PID) ---
KP = 0.25            # PID: 比例系数 (消除当前误差)
KD = 0.50            # PID: 微分系数 (抑制超调，提供阻尼)
MAX_STEER = 0.6      # 物理极限：最大方向盘转角 (约 35 度)
WHEELBASE = 2.8      # 车辆物理轴距 (米)


# =============================================================================
# [规控算法库]
# =============================================================================

def evaluate_lattice_cost(target_offset: float, obstacles: List[Dict[str, float]], last_offset: float) -> float:
    """
    @brief Lattice 代价寻优算法评估函数
    @param target_offset: 当前评估的候选横向偏移量
    @param obstacles: 局部坐标系下的障碍物列表
    @param last_offset: 上一帧的最优偏移量 (用于计算平滑度)
    @return float: 综合代价 (越小越好)
    """
    cost = 0.0
    
    # 1. 车道偏离代价 (趋向于回到 0.0 中心线)
    cost += W_LANE * abs(target_offset)
    
    # 2. 平滑度代价 (惩罚轨迹的突变)
    cost += W_SMOOTH * abs(target_offset - last_offset)
    
    # 3. 安全避障代价 (基于高斯风险场)
    for obs in obstacles:
        dx = obs.get("local_x", 0.0)
        dy = obs.get("local_y", 0.0) - target_offset 
        
        # 视场裁剪：忽略车身后方以及 40 米开外无关紧要的目标
        if dx < -(VEHICLE_LENGTH / 2.0 + 2.0) or dx > 40.0:
            continue
            
        # 计算车辆外边缘到障碍物质心的距离 (Edge Distance)
        edge_dx = max(0.0, abs(dx) - (VEHICLE_LENGTH / 2.0))
        edge_dy = max(0.0, abs(dy) - (VEHICLE_WIDTH / 2.0))
        
        # 绝对碰撞判定
        if edge_dx == 0.0 and edge_dy == 0.0:
            return float('inf') 
            
        # 计算高斯风险概率 P(risk) = exp(-(dx^2/2σx^2 + dy^2/2σy^2))
        risk = math.exp(-( (edge_dx**2)/(2 * SIGMA_X**2) + (edge_dy**2)/(2 * SIGMA_Y**2) ))
        
        # 截断极小风险，节省算力
        if risk > 0.01: 
            cost += W_COLLISION * risk
            
    return cost


def evaluate_apf_force(obstacles: List[Dict[str, float]], last_offset: float) -> float:
    """
    @brief 人工势场法 (APF)
    * 通过模拟物理电磁场中的“斥力”，将车辆推离障碍物。
    """
    force_y = 0.0
    
    # 引力场：始终有一个把车拉回车道中心 (0.0) 的虚拟弹簧力
    force_y -= last_offset * 0.5
    
    # 斥力场：障碍物推开车辆的力
    for obs in obstacles:
        dx = obs.get("local_x", 0.0)
        dy = obs.get("local_y", 0.0) - last_offset
        
        # 盲区过滤：忽略极近处的雷达噪点，以及太远的目标
        if 2.0 < dx < 30.0 and abs(dy) < 3.0:
            direction = -1.0 if dy > 0 else 1.0
            # 斥力强度与纵向距离成反比，与横向距离有关
            repulsive_strength = 40.0 / (dx * max(0.1, abs(dy)))
            force_y += direction * repulsive_strength
            
    # 计算受力后的预期目标，并进行物理道路宽度限幅
    target = last_offset + force_y * 0.1
    return max(-3.5, min(3.5, target))


def mpc_optimize_steering(ego_y: float, ego_yaw: float, target_y: float, speed: float) -> float:
    """
    @brief 简易版采样 MPC (Model Predictive Control)
    * 利用运动学自行车模型 (Bicycle Model)，推演未来 N 步的状态，在预测域中寻找最优方向盘转角。
    """
    best_steer = 0.0
    min_cost = float('inf')
    
    horizon = 10   # 预测未来 10 帧 (结合 dt=0.1，预测未来 1.0 秒)
    dt = 0.1
    
    # 动态采样 21 个可能的方向盘转角指令 (-MAX 到 +MAX 均匀分布)
    steer_candidates = [ -MAX_STEER + (2 * MAX_STEER / 20) * i for i in range(21) ]
    
    for steer in steer_candidates:
        pred_y = ego_y
        pred_yaw = ego_yaw
        cost = 0.0
        
        # 运动学状态推演方程 (State Transition Equation)
        for _ in range(horizon):
            yaw_rate = (speed / WHEELBASE) * math.tan(steer)
            pred_yaw += yaw_rate * dt
            pred_y += speed * math.sin(pred_yaw) * dt
            
            # 阶段代价 (Stage Cost)：横向位置误差惩罚 + 航向角未回正惩罚
            cost += 1.0 * (pred_y - target_y)**2 + 0.5 * (pred_yaw)**2
            
        if cost < min_cost:
            min_cost = cost
            best_steer = steer
            
    return best_steer


# =============================================================================
# [主控节点网络与生命周期]
# =============================================================================

def control_loop():
    logger.info("🤖 全链路规控大脑节点已启动 (集成 Lattice / APF / MPC+PID 引擎)...")
    
    # --- 1. 建立与 C++ 底座的 TCP 长连接 ---
    while True:
        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            client.connect(('127.0.0.1', 8080))
            logger.info("✅ 成功握手到底层 C++ 仿真引擎！等待数据流推送...")
            break  
        except ConnectionRefusedError:
            client.close()  
            logger.warning("⏳ 正在等待 C++ 引擎就绪 (请在 GUI 界面点击 '▶ 开始沙盒' 唤醒端口)...")
            time.sleep(2)
            
    buffer = ""
    last_best_offset = 0.0 
    last_error_y = 0.0 
    
    # --- 2. 实时业务循环 ---
    while True:
        try:
            data = client.recv(8192).decode('utf-8')
            if not data: 
                logger.error("❌ C++ 底座主动断开连接，规控大脑退出。")
                break
                
            buffer += data
            
            # 【TCP 粘包处理】利用换行符 \n 完美切分独立的数据帧
            while '\n' in buffer:
                line, buffer = buffer.split('\n', 1)
                if not line.strip(): continue
                
                try:
                    msg = json.loads(line)
                    if msg.get("type") == "OBSTACLES":
                        obstacles = msg.get("data", [])
                        plan_algo = msg.get("plan_algo", 0)
                        
                        # 解析闭环物理状态反馈
                        ego_y = msg.get("ego_y", 0.0)
                        ego_yaw = msg.get("ego_yaw", 0.0)
                        best_offset = 0.0
                        
                        # --- 安全兜底：碰撞警告哨兵 ---
                        for obs in obstacles:
                            if 2.0 < obs.get("local_x", 0) < 15.0 and abs(obs.get("local_y", 10)) < 2.0:
                                logger.warning(f"⚠️ 紧急避障触发！前方 {obs['local_x']:.1f}m 发现高危障碍物")
                                break
                        
                        # --- 路由到具体的算法执行器 ---
                        if plan_algo == 0:
                            # [算法 0]: Lattice 纯几何平移采样
                            min_cost = float('inf')
                            for offset in CANDIDATE_OFFSETS:
                                cost = evaluate_lattice_cost(offset, obstacles, last_best_offset)
                                if cost < min_cost:
                                    min_cost = cost
                                    best_offset = offset
                            if min_cost == float('inf'): 
                                best_offset = last_best_offset 
                                
                            last_best_offset = best_offset
                            cmd = {"type": "CONTROL", "steer_offset": best_offset}
                            
                        elif plan_algo == 1:
                            # [算法 1]: APF 人工势场法平移
                            best_offset = evaluate_apf_force(obstacles, last_best_offset)
                            last_best_offset = best_offset
                            cmd = {"type": "CONTROL", "steer_offset": best_offset}
                            
                        elif plan_algo == 2:
                            # [算法 2]: 物理级 MPC + PID 混合联合控制 (Feedforward + Feedback)
                            # 步骤 A：利用 Lattice 获取期望目标车道 target_y
                            min_cost = float('inf')
                            target_y = last_best_offset
                            for offset in CANDIDATE_OFFSETS:
                                cost = evaluate_lattice_cost(offset, obstacles, last_best_offset)
                                if cost < min_cost:
                                    min_cost = cost
                                    target_y = offset
                            if min_cost == float('inf'): 
                                target_y = last_best_offset
                            last_best_offset = target_y
                            
                            speed = 10.0 # 期望巡航速度 (m/s)
                            
                            # 步骤 B：前馈控制 (MPC Feedforward) -> 计算理想转角
                            mpc_steer = mpc_optimize_steering(ego_y, ego_yaw, target_y, speed)
                            
                            # 步骤 C：反馈控制 (PID Feedback) -> 修正环境扰动误差
                            error_y = target_y - ego_y
                            diff_error_y = error_y - last_error_y
                            pid_steer = KP * error_y + KD * diff_error_y
                            last_error_y = error_y
                            
                            # 步骤 D：控制量融合 (70% 预测信任 + 30% 误差补偿)
                            final_steer = 0.7 * mpc_steer + 0.3 * pid_steer
                            
                            # 执行器物理限幅保护
                            final_steer = max(-MAX_STEER, min(MAX_STEER, final_steer))
                            
                            cmd = {
                                "type": "CONTROL_KINEMATIC", 
                                "steering": final_steer,
                                "speed": speed 
                            }
                        else:
                            continue
                        
                        # --- 将控制指令序列化并下发给底座执行器 ---
                        client.sendall((json.dumps(cmd) + "\n").encode('utf-8'))
                        
                except json.JSONDecodeError:
                    logger.error("JSON 报文解析失败，已丢弃该帧。")
                    
        except ConnectionResetError:
            logger.error("❌ C++ 仿真底座异常崩溃或断开连接。")
            break

if __name__ == "__main__":
    try:
        control_loop()
    except KeyboardInterrupt:
        logger.info("🛑 接收到退出信号，规控大脑安全关闭。")
