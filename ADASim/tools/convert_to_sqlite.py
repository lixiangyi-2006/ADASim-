#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@file convert_to_sqlite.py
@brief 自动驾驶算法仿真平台 (ADASim) - 数据集打包与合成工具
@date 2026-04

@details
本脚本负责构建 C++ 引擎所需的高性能 SQLite 离线数据库。
* 【架构与性能优化说明】
1. 跨平台二进制兼容：在 struct.pack 中强制使用 '<' (Little-Endian 小端序)，确保 Python 写入的 BLOB 数据在 C++ 引擎进行 reinterpret_cast 时绝对安全，防止出现坐标乱码（即跨平台架构兼容）。
2. 无缝降级机制 (Fallback)：当真实的 JSON/BIN 数据集缺失时，自动降级为生成 100 帧完美的 360° 均匀雷达圆环沙盒数据，保证系统开箱即用。
3. 事务安全：使用 SQLite 事务提交机制，防止中途断电或异常导致的数据库损坏。
"""

import sqlite3
import json
import struct
import os
import math

# =============================================================================
# [全局路径配置]
# =============================================================================
DATA_DIR = 'data'
DB_PATH = os.path.join(DATA_DIR, 'dataset.db')
TRAJ_PATH = os.path.join(DATA_DIR, 'trajectory.json')
LIDAR_DIR = os.path.join(DATA_DIR, 'lidar')

def build_db():
    print("🚀 [ADASim] 开始构建高性能 SQLite 数据引擎...")
    
    # 1. 确保数据目录存在
    os.makedirs(DATA_DIR, exist_ok=True)
    
    # 2. 清理残缺或旧版数据库，确保每次生成的都是干净的
    if os.path.exists(DB_PATH):
        os.remove(DB_PATH)
        print(f"🗑️ 已清理旧版本数据库: {DB_PATH}")
        
    # 3. 建立数据库连接
    conn = sqlite3.connect(DB_PATH)
    c = conn.cursor()
    
    # 4. 创建满血版 frames 核心表
    # x, y, yaw: 车辆在世界坐标系下的绝对位姿
    # point_cloud: 当前帧激光雷达点云，为追求极致读取性能，直接采用 BLOB 二进制存储
    c.execute('''CREATE TABLE frames (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        x REAL, 
        y REAL, 
        yaw REAL, 
        point_cloud BLOB
    )''')

    # =========================================================================
    # [数据组装与注入逻辑]
    # =========================================================================
    if os.path.exists(TRAJ_PATH):
        print(f"📂 发现本地真实轨迹文件: {TRAJ_PATH}，正在启动转换流水线...")
        with open(TRAJ_PATH, 'r', encoding='utf-8') as f:
            traj = json.load(f)
            
        for i, pt in enumerate(traj):
            bin_path = os.path.join(LIDAR_DIR, f"frame_{i}.bin")
            blob = b''
            
            # 读取对应的雷达点云二进制文件
            if os.path.exists(bin_path):
                with open(bin_path, 'rb') as f_bin:
                    blob = f_bin.read()
            else:
                print(f"⚠️ 警告: 第 {i} 帧缺失雷达数据 ({bin_path})")
                
            c.execute('INSERT INTO frames (x, y, yaw, point_cloud) VALUES (?, ?, ?, ?)',
                      (pt.get('x', 0.0), pt.get('y', 0.0), pt.get('yaw', 0.0), blob))
    else:
        print("⚙️ 未找到 JSON 真实轨迹，正在生成 100 帧高精度【纯净沙盒测试数据】...")
        for i in range(100):
            # 模拟车辆沿 X 轴匀速直线行驶 (约合 18km/h)
            x = i * 0.2
            y = 0.0
            yaw = 0.0
            points = []
            
            # 模拟一个完美的 36 线单圈雷达扫描 (每 10 度一个点，距离车心 5 米)
            for j in range(36):
                angle = j * 10 * math.pi / 180.0
                points.extend([5.0 * math.cos(angle), 5.0 * math.sin(angle)])
                
            # 【核心架构安全】：使用 '<' 强制小端序打包！
            # 格式串说明: '<' 代表小端序，'d' 代表 8字节 double 浮点数
            # 这能确保 C++ 端用 double* 解析时，绝不会因为 CPU 架构不同而读出天文数字
            pack_format = f'<{len(points)}d'
            blob = struct.pack(pack_format, *points)
            
            c.execute('INSERT INTO frames (x, y, yaw, point_cloud) VALUES (?, ?, ?, ?)',
                      (x, y, yaw, blob))
                      
    # 5. 提交事务并安全关闭连接
    conn.commit()
    conn.close()
    
    print(f"✅ [ADASim] 数据锻造完成！完美的数据库已就绪: {DB_PATH}")
    print("   ↳ 表名: frames | 包含点云 BLOB 字段 | 准备交由 C++ 引擎极速渲染")

if __name__ == '__main__':
    try:
        build_db()
    except Exception as e:
        print(f"❌ 数据库构建发生致命错误: {e}")