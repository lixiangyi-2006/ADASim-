/**
 * @file DataLoader.h
 * @brief 自动驾驶算法仿真平台 (ADASim) - 数据流引擎与沙盒控制头文件
 * @date 2026-04
 * @details
 * 本文件定义了负责数据流转、沙盒模拟与时间轴控制的核心引擎 `DataLoader`。
 * * * 【说明】
 * 1. 线程隔离：本类设计为在独立的 Backend 子线程中运行，绝不允许任何阻塞 GUI 主线程的操作。
 * 2. 内存管理清理：彻底移除了早期废弃的 SQLite (QSqlDatabase) 残留变量，保持类的轻量化与职责单一。
 * 3. 滚动沙盒模式 (Rolling Window Replay Buffer)：抛弃了固定长度的旧模式，
 * 采用“无限向前的真实物理时间 + 定长(100帧)的环形队列”实现行车记录仪级别的回溯功能。
 * 4. 智能时间分流：通过引入 `replayCursor_` 实现了“未来生成模式”与“历史回放模式”的无缝安全切换。
 */

#ifndef DATALOADER_H
#define DATALOADER_H

#include "RingBuffer.h"

#include <QObject>
#include <QVector>
#include <QPointF>
#include <QTimer>
#include <chrono>
#include <cmath>
#include <random>

/**
 * @class DataLoader
 * @brief 高频数据泵、沙盒生成器与时间轴回溯引擎
 * * 负责按特定频率（如 10Hz）生成自车位姿与传感器点云，
 * 并通过 Qt 跨线程信号槽机制，分发给 UI 渲染层与算法中枢。
 */
class DataLoader : public QObject
{
    Q_OBJECT
public:
    explicit DataLoader(const QString& dataPath, QObject *parent = nullptr);
    ~DataLoader();
    

signals:
    // ==========================================
    // [感知与规控核心数据信号]
    // ==========================================
    
    /**
     * @brief 点云数据就绪信号
     * @param points 当前帧的 2D 激光雷达点云集合
     * * 💡【拓展功能接口】：未来若接入真实的 3D LiDAR (如速腾/禾赛的 32线/64线雷达)，
     * 可将本信号的参数升级为 ROS/PCL 标准的 `pcl::PointCloud<pcl::PointXYZ>::Ptr`，实现 3D 渲染对接。
     */
    void pointCloudReady(const QVector<QPointF>& points);
    
    /**
     * @brief 车辆自身物理状态就绪信号 (Ego State)
     * @param x 车辆全局 X 坐标 (米)
     * @param y 车辆全局 Y 坐标 (米)
     * @param yaw 车辆全局航向角 (弧度)
     * * 💡【拓展功能接口】：随着物理引擎的升级，可在此新增速度(vx, vy)、加速度(ax, ay)和方向盘转角(steering)，以支持更精密的 MPC 闭环控制。
     */
    void vehiclePositionReady(double x, double y, double yaw, double dt);
    
    /**
     * @brief 引擎状态通知信号，用于更新界面的底部状态栏
     */
    void statusUpdate(const QString& status);
    
    // ==========================================
    // [时间进度条双向绑定信号]
    // ==========================================
    
    void totalFramesLoaded(int total);    ///< 通知 UI 更新进度条的动态总上限（FIFO 队列当前大小）
    void currentFrameUpdated(int index);  ///< 通知 UI 当前播放的帧索引，驱动进度条游标前进

public slots:
    void start();                         ///< 启动引擎（重置历史记录并开始生成无限沙盒数据）
    void pause();                         ///< 暂停沙盒时间轴
    void stop();                          ///< 停止引擎并清理状态
    
    /**
     * @brief 接收 UI 进度条的拖拽跳转指令，实现“时光倒流”
     * @param index 目标历史帧在 FIFO 队列中的索引
     */
    void seekToFrame(int index);          

private slots:
    /**
     * @brief 定时器高频触发的核心数据泵（Data Pump）事件
     * * 内部包含智能分流逻辑：判断当前是应该播放历史缓存，还是生成未来的新数据。
     */
    void loadNextFrame();

private:
    /**
     * @struct FrameState
     * @brief 历史状态帧轻量级封装，用于 FIFO 回放队列的高效存储
     */
    struct FrameState {
        double x;
        double y;
        double yaw;
    };

    // ==========================================
    // [底层状态维护区]
    // ==========================================
    RingBuffer<FrameState> historyQueue_;  ///< 记录最近 100 帧状态的 FIFO 滚动缓冲区 (Rolling Window)
    double liveSandboxX_ = 0.0;         ///< 记录沙盒模式下无限向前的真实物理世界 X 坐标
    
    /**
     * @brief 智能回放游标
     * * 状态机制：
     * -1：表示当前追上现实，正在探索未来（生成新数据）
     * >=0：表示用户拖拽了进度条，正在播放 FIFO 队列中的历史时刻
     */
    int replayCursor_ = -1;             
    
    QString dataPath_;                  ///< 数据目录路径（保留用于拓展离线高精地图加载）
    bool isRunning_ = false;            ///< 引擎运行状态标记
    
    QTimer* timer_;                     ///< 高频数据驱动核心定时器
    
    std::chrono::steady_clock::time_point lastTimestamp_; 
    std::default_random_engine rng_;
    std::normal_distribution<double> noise_dist_; 
};

#endif // DATALOADER_H
