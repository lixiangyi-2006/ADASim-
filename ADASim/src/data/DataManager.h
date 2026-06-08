/**
 * @file DataManager.h
 * @brief 自动驾驶算法仿真平台 (ADASim) - 数据总线与逻辑中枢头文件
 * @date 2026-04
 * @details
 * 本模块充当系统的中介者 (Mediator)，负责隔离 UI 渲染层与底层算法/网络层。
 * * 【架构与设计规范】
 * 1. 数据路由：统一接管来自 DataLoader(沙盒数据) 和 SocketServer(外部网络) 的输入，处理后再分发给 UI 视图。
 * 2. 跨线程注册：显式注册了 Q_DECLARE_METATYPE，保障复杂的算法产物 (如 PredictedPath) 能够在 Qt 的多线程环境下游刃有余地传递。
 * 3. 雷达欺骗引擎：内置了将全局沙盒 UI 操作逆向转化为底层传感器点云的逻辑架构。
 */

#ifndef DATAMANAGER_H
#define DATAMANAGER_H

#include <QObject>
#include <QVector>
#include <QPointF>
#include <QByteArray>
#include <QString> // 新增：修复 QString status 的编译依赖

// 包含具体的算法模块
#include "algorithm/ObstacleDetector.h"
#include "algorithm/PathPredictor.h"

// ============================================================================
// [跨线程元类型注册] 允许将算法产出通过信号槽在多线程间安全排队 (QueuedConnection)
// ============================================================================
Q_DECLARE_METATYPE(QVector<Obstacle>)
Q_DECLARE_METATYPE(PredictedPath)

/**
 * @class DataManager
 * @brief 平台核心数据总线、算法调度器与沙盒交互中枢
 */
class DataManager : public QObject
{
    Q_OBJECT
public:
    explicit DataManager(QObject* parent = nullptr);
    ~DataManager();

public slots:
    // ==========================================
    // [数据输入层 - Inbound]
    // ==========================================
    
    // 1. 接收本地原始物理推演数据 (Sender: DataLoader / MainWindow)
    void onPointCloudReceived(const QVector<QPointF>& points);
    void onVehiclePositionReceived(double x, double y, double yaw);
    
    // 2. 接收“上帝之手”沙盒注入 (Sender: View2D)
    void onUserObstacleAdded(double x, double y); 
    
    // 3. 接收网络外部数据 (Sender: SocketServer)
    // 💡【拓展功能接口】：未来如果接入 ROS/ROS2 桥接层，可在此处新增 void onRosMessageReceived(const QByteArray& msg);
    void onNetworkDataReceived(const QByteArray& data);
    
    // 4. 接收 UI 控制指令配置算法参数 (Sender: MainWindow)
    void onAlgorithmChanged(int index);         ///< 切换感知/聚类模型
    void onPlanAlgorithmChanged(int index);     ///< 切换规控模型

signals:
    // ==========================================
    // [数据输出层 - Outbound]
    // ==========================================
    
    // 1. 发送给 UI 渲染的算法产物 (Receiver: View2D / SensorView)
    void obstaclesDetected(const QVector<QPointF>& obstaclePositions); ///< 提取出的障碍物质心坐标 (全局)
    void pathPredicted(const QVector<QPointF>& predictedTrajectory);   ///< 预测出的未来轨迹点列
    void mergedPointCloudReady(const QVector<QPointF>& points);        ///< 融合了沙盒欺骗数据后的最终雷达点云
    
    // 2. 发送给网络的广播数据 (Receiver: SocketServer)
    void broadcastDataRequested(const QByteArray& data);
    
    // 3. 状态与控制指令上报 (Receiver: MainWindow)
    void algorithmStatusUpdated(const QString& status);                ///< 算法运行状态反馈
    void lateralControlReceived(double offset);                        ///< 纯几何横向避障指令透传
    void kinematicControlReceived(double steeringAngle, double targetSpeed); ///< 动力学控制指令透传
    
private:
    // ==========================================
    // [内部算法引擎池]
    // ==========================================
    ObstacleDetector obstacleDetector_;        ///< 雷达点云感知与聚类引擎
    PathPredictor pathPredictor_;              ///< 运动学轨迹预测引擎
    
    // ==========================================
    // [系统运行缓存状态区]
    // * 严格使用 Modern C++ 原地初始化，杜绝脏数据
    // ==========================================
    QVector<QPointF> historyTrajectory_;       ///< 缓存本车历史轨迹 (供预测模块推演)
    QVector<QPointF> userObstacles_;           ///< 缓存用户在沙盒中手动添加的真值障碍物
    
    int currentAlgoIndex_ = 0;                 ///< 当前激活的感知算法模式 ID (0:聚类, 1:RANSAC, 2:YOLO)
    int currentPlanAlgoIndex_ = 0;             ///< 当前激活的规控算法模式 ID (0:Lattice, 1:APF, 2:MPC/PID)
    
    // 自车上一帧的绝对物理坐标 (用于雷达欺骗的坐标系逆变换)
    double lastEgoX_ = 0.0;
    double lastEgoY_ = 0.0;
    double lastEgoYaw_ = 0.0;
};

#endif // DATAMANAGER_H