/**
 * @file DataManager.cpp
 * @brief 自动驾驶算法仿真平台 (ADASim) - 数据总线与逻辑中枢实现
 * @date 2026-04
 * @details
 * 本文件是连接 C++ 底层物理世界与 Python 高层规控大脑的“桥梁 (Middleware)”。
 * * 【架构与性能说明】
 * 1. 雷达欺骗引擎 (Sandbox Injection)：实现了将 UI 全局物理坐标逆向映射为自车局部坐标系，并生成虚拟雷达点云，实现算法的闭环沙盒测试。
 * 2. 极致数学优化 (Math Optimization)：将高耗时的三角函数 std::cos/sin 从十万次级的点云遍历循环中提取到外部，每帧仅计算一次，极大释放了 CPU 算力。
 * 3. 内存搬移优化：在历史轨迹队列 (historyTrajectory_) 和特征过滤操作中预分配内存，减少 QVector 的堆内存扩容开销。
 */

#include "DataManager.h"
#include "common/Logger.h" // 接入 ADASim 全局日志系统

#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <cmath> 

// ============================================================================
// [构造与析构]
// ============================================================================

DataManager::DataManager(QObject* parent) 
    : QObject(parent)
{
    // 配置感知与预测算法的底层物理参数
    obstacleDetector_.setThreshold(40.0f, 5);  // 感知阈值：40米内，至少包含5个雷达点的簇才算有效障碍物
    pathPredictor_.setParameters(10.0f, 0.0f); // 预测频率：10Hz
    
    // 【跨线程基石】：注册自定义类型的元数据，允许它们在不同 QThread 间通过信号槽安全传递
    qRegisterMetaType<QVector<Obstacle>>("QVector<Obstacle>");
    qRegisterMetaType<PredictedPath>("PredictedPath");
    
    LOG_INFO("DataManager: 数据中枢总线已初始化完毕");
}

DataManager::~DataManager() {}

// ============================================================================
// [感知管线：点云接收与沙盒注入]
// ============================================================================

void DataManager::onPointCloudReceived(const QVector<QPointF>& points)
{
    QVector<QPointF> mergedPoints = points;
    
    // 【极致数学优化】：当前帧的偏航角是不变的，提前计算好旋转矩阵的三角函数值。
    // 杜绝在下方遍历数万个点云和生成沙盒点时重复计算！
    const double cosYaw = std::cos(lastEgoYaw_);
    const double sinYaw = std::sin(lastEgoYaw_);
    
    // ==========================================
    // 1. 【雷达沙盒欺骗】(Radar Spoofing)
    // ==========================================
    // 将用户用鼠标在屏幕上点击的“全局障碍物”，逆向转换到“自车局部坐标系”，并生成虚拟回波点
    for (const auto& obs : userObstacles_) {
        double dx = obs.x() - lastEgoX_;
        double dy = obs.y() - lastEgoY_;
        
        // 旋转矩阵：全局坐标系 -> 局部坐标系
        double localX = dx * cosYaw + dy * sinYaw;
        double localY = -dx * sinYaw + dy * cosYaw;
        
        // 剔除 40 米有效射程之外的目标 (40^2 = 1600)
        if (localX * localX + localY * localY < 1600.0) {
            // 环绕该坐标生成 36 个雷达反射点，模拟真实的柱状障碍物反射
            for(int i = 0; i < 36; i++) {
                double angle = i * 10 * M_PI / 180.0;
                mergedPoints.append(QPointF(localX + 0.5 * std::cos(angle), localY + 0.5 * std::sin(angle)));
            }
        }
    }
    
    // ==========================================
    // 2. 【传感器算法特征模拟】(Sensor Logic Simulation)
    // ==========================================
    QVector<QPointF> filteredPoints;
    
    if (currentAlgoIndex_ == 1) {
        // 模式 1: RANSAC 算法模拟
        // 特性：强力过滤掉距离雷达极近（<2米）的车身反光或地面噪点
        filteredPoints.reserve(mergedPoints.size());
        for (const auto& pt : mergedPoints) {
            if (pt.x()*pt.x() + pt.y()*pt.y() > 4.0) { 
                filteredPoints.append(pt);
            }
        }
    } else {
        // 模式 0 / 2: 基础聚类，不过滤原始点云
        filteredPoints = mergedPoints;
    }

    // ==========================================
    // 3. 【核心感知执行】
    // ==========================================
    QVector<Obstacle> obstacles = obstacleDetector_.detect(filteredPoints);
    
    if (currentAlgoIndex_ == 2) {
        // 模式 2: YOLO 语义识别模型模拟
        // 特性：过滤掉路边琐碎的非车/非人目标，只关注距离大于 3m 的明确结构化障碍物
        QVector<Obstacle> yoloObstacles;
        for(const auto& obs : obstacles) {
            if(obs.distance > 3.0f) {
                yoloObstacles.append(obs);
            }
        }
        obstacles = std::move(yoloObstacles); // 使用 move 语义，避免深拷贝
    }
    
    // ==========================================
    // 4. 【坐标回传与 Python 通信包组装】
    // ==========================================
    QVector<QPointF> obsPositions;
    QJsonArray obsArrayForPython;
    
    for (const auto& obs : obstacles) {
        // 局部转全局，发给 View2D (BEV上帝视角) 画红色报警圈
        double globalX = lastEgoX_ + obs.position.x() * cosYaw - obs.position.y() * sinYaw;
        double globalY = lastEgoY_ + obs.position.x() * sinYaw + obs.position.y() * cosYaw;
        obsPositions.append(QPointF(globalX, globalY));
        
        // 将局部坐标系数据打包，准备发给 Python 的 Lattice/MPC 大脑
        QJsonObject o;
        o["dist"] = obs.distance;
        o["local_x"] = obs.position.x();
        o["local_y"] = obs.position.y();
        obsArrayForPython.append(o);
    }
    
    // ==========================================
    // 5. 【数据分发】
    // ==========================================
    emit obstaclesDetected(obsPositions);         // 分发给 View2D 绘制宏观障碍物
    emit mergedPointCloudReady(mergedPoints);     // 分发给 SensorView 绘制第一人称点云

    // 组装最终规控心跳包 (Control Heartbeat Packet)
    QJsonObject out;
    out["type"] = "OBSTACLES";
    out["data"] = obsArrayForPython;
    out["plan_algo"] = currentPlanAlgoIndex_;
    out["ego_y"] = lastEgoY_;
    out["ego_yaw"] = lastEgoYaw_;
    
    // 遵循流式通信协议，加上 '\n' 作为 TCP 粘包的切割符
    emit broadcastDataRequested(QJsonDocument(out).toJson(QJsonDocument::Compact) + "\n");
}

// ============================================================================
// [状态管线：自车位姿与轨迹预测]
// ============================================================================

void DataManager::onVehiclePositionReceived(double x, double y, double yaw)
{
    lastEgoX_ = x; 
    lastEgoY_ = y; 
    lastEgoYaw_ = yaw;
    
    // 1. 维护历史轨迹 FIFO 队列 (用于轨迹推演与预测)
    historyTrajectory_.append(QPointF(x, y));
    if (historyTrajectory_.size() > 200) {
        historyTrajectory_.pop_front(); // 严格保持 200 帧上限
    }
    
    // 2. 驱动预测算法引擎 (预测未来 3 秒的动向)
    PredictedPath pred = pathPredictor_.predict(historyTrajectory_, 3);
    emit pathPredicted(pred.path);
    
    // 3. 【网络联动】：广播当前车辆绝对物理状态 (Ego State)
    QJsonObject posObj;
    posObj["type"] = "EGO_STATE";
    posObj["x"] = x;
    posObj["y"] = y;
    posObj["yaw"] = yaw;
    
    QJsonDocument doc(posObj);
    emit broadcastDataRequested(doc.toJson(QJsonDocument::Compact) + "\n");
}

// ============================================================================
// [网络管线：接收远端算法指令]
// ============================================================================

void DataManager::onNetworkDataReceived(const QByteArray& data)
{
    // 【TCP 粘包解析】：远端发来的数据流可能包含多条指令，严格通过换行符切片
    QList<QByteArray> lines = data.split('\n');
    
    for (const QByteArray& line : lines) {
        if (line.trimmed().isEmpty()) continue; 
        
        // JSON 反序列化
        QJsonParseError error;
        QJsonDocument doc = QJsonDocument::fromJson(line, &error);
        
        if (doc.isObject()) {
            QJsonObject obj = doc.object();
            
            // 模式 A：几何平移控制指令 (Lattice / APF)
            if (obj["type"] == "CONTROL") {
                emit lateralControlReceived(obj["steer_offset"].toDouble());
            } 
            // 模式 B：动力学物理闭环控制指令 (MPC / PID)
            else if (obj["type"] == "CONTROL_KINEMATIC") {
                double steering = obj["steering"].toDouble();
                double speed = obj["speed"].toDouble();
                emit kinematicControlReceived(steering, speed);
            }
        } else {
            LOG_WARN("DataManager: 收到格式异常的 JSON 控制报文: %s", line.data());
        }
    }
}

// ============================================================================
// [UI 交互响应]
// ============================================================================

void DataManager::onUserObstacleAdded(double x, double y)
{
    userObstacles_.append(QPointF(x, y));
    LOG_INFO("DataManager: 上帝之手注入实体障碍物 -> 全局坐标: (X:%.2f, Y:%.2f)", x, y);
}

void DataManager::onAlgorithmChanged(int index)
{
    currentAlgoIndex_ = index;
    QString status;
    
    // 模拟不同感知算法的计算延迟与特征
    if (index == 0) {
        status = "✅ 点云聚类算法就绪 | 延迟：<5ms | 适合低速通用场景";
    } else if (index == 1) {
        status = "✅ RANSAC 去地噪就绪 | 延迟：<15ms | 强抗噪，忽略小杂散点";
    } else {
        status = "✅ YOLO 语义模拟就绪 | 延迟：<25ms | 只响应真实语义大目标";
    }
    emit algorithmStatusUpdated(status);
    LOG_INFO("DataManager: 切换感知算法至模式 %d", index);
}

void DataManager::onPlanAlgorithmChanged(int index)
{
    currentPlanAlgoIndex_ = index;
    LOG_INFO("DataManager: 切换规控算法至模式 %d", index);
}