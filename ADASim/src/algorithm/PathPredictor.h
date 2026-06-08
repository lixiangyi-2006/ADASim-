/**
 * @file PathPredictor.h
 * @brief 自动驾驶算法仿真平台 (ADASim) - 轨迹预测模块头文件
 * @date 2026-04
 * @details
 * 本文件定义了障碍物未来行驶轨迹的预测算法。当前版本实现了基础的“匀速运动学模型 (Constant Velocity Model)”。
 * * 【说明】
 * 1. 内存安全：为结构体与类成员全部添加了 C++11 缺省初始化，杜绝规控端读取到未初始化的“乱码”轨迹。
 * 2. 契约式设计：引入 [[nodiscard]]，强制要求调用端必须接收预测结果，防止 CPU 算力白白浪费。
 * 3. 极简抽象：对外隐藏了复杂的运动推演逻辑，仅暴露 `predict` 极简接口，方便后期无缝替换底层预测算法。
 */

#ifndef PATHPREDICTOR_H
#define PATHPREDICTOR_H

#include <QVector>
#include <QPointF>

/**
 * @struct PredictedPath
 * @brief 结构化预测轨迹
 * * 封装了对某一动态障碍物在未来一段时间内的行为预测结果。
 */
struct PredictedPath
{
    // ==========================================
    // [预测状态数据区] 缺省初始化，消灭野数据
    // ==========================================
    QVector<QPointF> path;          ///< 预测出的未来轨迹点列
    float confidence = 0.0f;        ///< 预测置信度 (0.0 ~ 1.0)，如轨迹跳变严重则置信度降低
    int horizon = 0;                ///< 预测时域 / 预测步数 (如 30 步代表未来 3 秒)
    
    // ==========================================
    // 💡【拓展功能接口】：高级预测算法扩充
    // ==========================================
    // QVector<float> uncertainties; ///< [拓展] 每个预测点的横向误差/协方差 (用于绘制喇叭状的预测概率管 / Uncertainty Tube)
    // int intent = 0;               ///< [拓展] 行为意图枚举 (0:保持直行, 1:左变道, 2:右变道, 3:急刹)
};

/**
 * @class PathPredictor
 * @brief 障碍物意图与轨迹预测引擎
 */
class PathPredictor
{
public:
    PathPredictor();
    ~PathPredictor();
    
    /**
     * @brief 核心预测管线：基于历史轨迹推演未来时态
     * @param trajectory 障碍物的历史行驶轨迹 (传入 const 引用，零拷贝)
     * @param seconds 期望预测的未来时间跨度 (默认预测未来 3 秒)
     * @return PredictedPath 包含未来点列与置信度的预测结果
     * @note 采用 [[nodiscard]] 强制调用者接管计算结果
     */
    [[nodiscard]] PredictedPath predict(const QVector<QPointF>& trajectory, int seconds = 3);
    
    /**
     * @brief 动态配置预测引擎的物理参数
     * @param frequency 系统运行频率 (Hz，如 10.0 代表每秒 10 帧)
     * @param speed 障碍物当前的瞬时标量速度 (m/s)
     */
    void setParameters(float frequency, float speed);
    
private:
    /**
     * @brief 匀速运动学推演模型 (CV Model - Constant Velocity)
     * * 假设目标在未来一段时间内保持当前的速度矢量（大小和方向）不变。
     * @param lastPoint 障碍物当前（最后已知）的位置
     * @param velocity 障碍物当前的速度矢量 (Vx, Vy)
     * @param futureTime 预测的未来时间点 (秒)
     * @return QPointF 在 futureTime 时刻的预测坐标
     * * * 💡【架构演进提示】：
     * 真实的自动驾驶预测往往需要引入车辆转角，未来可在此扩充：
     * QPointF ctrvModel(...) // CTRV (匀速恒定转弯率模型 - Constant Turn Rate and Velocity)
     * QPointF ctraModel(...) // CTRA (恒定加速度与恒定转弯率模型)
     */
    QPointF constantVelocityModel(const QPointF& lastPoint, 
                                  const QPointF& velocity, 
                                  float futureTime);
    
private:
    // ==========================================
    // [引擎配置参数]
    // ==========================================
    float frequency_ = 10.0f;       ///< 系统高频滴答率 (默认 10Hz)
    float speed_ = 0.0f;            ///< 外部注入的目标速度缓冲 (m/s)
};

#endif // PATHPREDICTOR_H