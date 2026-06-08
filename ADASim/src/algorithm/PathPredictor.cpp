/**
 * @file PathPredictor.cpp
 * @brief 自动驾驶算法仿真平台 (ADASim) - 轨迹预测算法实现
 * @date 2026-04
 * @details
 * 本文件实现了基于车辆历史轨迹的未来状态推演。当前采用匀速运动学模型 (Constant Velocity Model)。
 * * 【架构与性能调优说明】
 * 1. 物理量纲纠正：严格遵循 v = Δs / Δt，修复了原版中将“单帧位移”直接作为“速度”的致命物理错误。
 * 2. 消除数值溢出隐患：采用 std::hypot(x, y) 替代 std::sqrt(x*x + y*y)，避免在极端坐标系下的浮点溢出/下溢。
 * 3. 内存友好：通过 reserve() 对预测路径所需的内存进行精准预分配，避免推演循环中发生深拷贝扩容。
 * 4. 死代码清除：彻底剔除了遗留且无用的线性回归 (Linear Regression) 废代码。
 */

#include "PathPredictor.h"
#include <cmath>
#include <QtGlobal>

// ============================================================================
// [构造与析构]
// ============================================================================
PathPredictor::PathPredictor()
    : frequency_(10.0f) // 系统默认滴答率为 10Hz (即每 0.1 秒一帧)
    , speed_(0.0f)
{
}

PathPredictor::~PathPredictor() {}

// ============================================================================
// [核心预测流水线]
// ============================================================================
PredictedPath PathPredictor::predict(const QVector<QPointF>& trajectory, int seconds)
{
    PredictedPath result;
    result.horizon = seconds;
    
    // 【边界防御】：至少需要 2 个历史点才能计算出速度矢量 (差分计算)
    if (trajectory.size() < 2) {
        result.confidence = 0.0f;
        return result;
    }
    
    // 提取最后两个时刻的有效观测点
    const QPointF& lastPoint = trajectory.last();
    const QPointF& prevPoint = trajectory[trajectory.size() - 2];
    
    // 【物理量纲恢复】：速度 = 位移 / 时间差 (dt)
    // 保护性逻辑：防止 frequency_ 为 0 导致浮点异常 (Divide by Zero)
    float safeFreq = (frequency_ > 0.001f) ? frequency_ : 10.0f;
    float dt = 1.0f / safeFreq;
    
    // 计算当前瞬时速度矢量 (Vx, Vy)
    QPointF velocity(
        (lastPoint.x() - prevPoint.x()) / dt,
        (lastPoint.y() - prevPoint.y()) / dt
    );
    
    // 【数值稳定性优化】：使用 std::hypot 计算欧几里得范数（标量速度）
    // 它在底层做了防溢出处理，比直接用 std::sqrt(x*x + y*y) 更安全、更专业
    speed_ = std::hypot(velocity.x(), velocity.y());
    
    // 计算未来需要推演的总步数
    int totalSteps = seconds * static_cast<int>(safeFreq);
    
    // 【内存分配优化】：精准预留空间，防止 push_back/append 引发内存搬移
    result.path.reserve(totalSteps);
    
    // 向未来进行时序推演
    for (int step = 1; step <= totalSteps; ++step) {
        float futureTime = step * dt;
        QPointF predicted = constantVelocityModel(lastPoint, velocity, futureTime);
        result.path.append(predicted);
    }
    
    // ==========================================
    // 启发式预测置信度评估 (Heuristic Confidence)
    // ==========================================
    // 1. 历史数据越丰满，推演越可靠（50 帧约 5 秒的历史作为基准满分）
    float historyFactor = qMin(1.0f, trajectory.size() / 50.0f);
    
    // 2. 动静状态抑制：如果速度极小（< 0.5m/s），视为静止或雷达漂移，降低置信度防止“幽灵预测线”
    float speedFactor = (speed_ > 0.5f) ? 1.0f : 0.5f; 
    
    result.confidence = historyFactor * speedFactor;
    
    return result;
}

// ============================================================================
// [参数配置接口]
// ============================================================================
void PathPredictor::setParameters(float frequency, float speed)
{
    // 【防御性编程】：拒绝非法频率，防止后续产生无限大的 dt
    frequency_ = (frequency > 0.001f) ? frequency : 10.0f; 
    speed_ = speed;
}

// ============================================================================
// [底层运动学模型]
// ============================================================================
QPointF PathPredictor::constantVelocityModel(const QPointF& lastPoint,
                                             const QPointF& velocity,
                                             float futureTime)
{
    // 匀速运动学模型 (CV Model - Constant Velocity)
    // 状态转移方程：P(t) = P_0 + V * t
    float predictedX = lastPoint.x() + velocity.x() * futureTime;
    float predictedY = lastPoint.y() + velocity.y() * futureTime;
    
    return QPointF(predictedX, predictedY);
}