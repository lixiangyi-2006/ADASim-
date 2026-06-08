/**
 * @file SensorView.h
 * @brief 自动驾驶算法仿真平台 (ADASim) - 传感器感知可视化模块
 * @date 2026-04
 * @details
 * 本文件定义了负责独立渲染底层传感器（如 LiDAR 激光雷达）原始数据的可视化面板。
 * * 【架构设计与渲染规范】
 * 1. 视场隔离：本组件作为一个独立的 Widget，专门用于渲染“机器眼中”的世界（点云），与主视图（View2D，用于渲染宏观的规控轨迹）在物理和逻辑上完全隔离。
 * 2. 性能考量：当前基于 Qt 的 QPainter 实现 2D 软渲染。当点云数量激增至十万级时，可平滑迁移至 QOpenGLWidget 进行 GPU 硬件加速。
 */

#ifndef SENSORVIEW_H
#define SENSORVIEW_H

#include <QWidget>
#include <QVector>
#include <QPointF>

/**
 * @class SensorView
 * @brief 传感器感知数据独立视图组件
 * * 接收并实时渲染激光雷达 (LiDAR) 等传感器扫描到的原始点云数据。
 */
class SensorView : public QWidget
{
    Q_OBJECT
    
public:
    /**
     * @brief 构造函数，初始化感知视图的 UI 属性
     * @param parent 父窗口指针
     */
    explicit SensorView(QWidget *parent = nullptr);
    
    /**
     * @brief 析构函数，安全释放视图资源
     */
    ~SensorView();
    
public slots:
    /**
     * @brief [核心数据槽] 接收底层引擎或算法端发来的最新一帧点云数据
     * @param points 当前帧的 2D 激光雷达点云集合 (全局坐标或自车相对坐标)
     * * 💡【拓展功能接口】：未来如果要实现多传感器融合可视化，可以新增槽函数：
     * - void updateCameraImage(const QImage& img); // 摄像头画面
     * - void updateRadarTargets(const QVector<RadarTarget>& targets); // 毫米波雷达目标
     */
    void updatePointCloud(const QVector<QPointF>& points);
    
protected:
    /**
     * @brief 重写 Qt 核心重绘事件
     * * 当调用 update() 或窗口状态改变时，由 Qt 事件循环自动触发此函数进行画面渲染。
     * @param event 绘图事件对象
     */
    void paintEvent(QPaintEvent* event) override;
    
private:
    /**
     * @brief 激光雷达渲染子程序
     * * 负责具体的 QPainter 坐标系变换、画笔设置及打点操作。
     * @param painter 从 paintEvent 传递过来的绘图引擎引用
     * * 💡【拓展功能接口】：若需引入 Z 轴（高度）数据渲染为 3D 效果，
     * 可在此处根据点的 Z 坐标(高度/强度)动态修改画笔的颜色（伪彩渲染）。
     */
    void drawLidarView(QPainter& painter);
    
private:
    // ==========================================
    // [渲染缓存区]
    // ==========================================
    /**
     * @brief 缓存的最新一帧点云数据
     * * 为了避免高频数据刷新引发的 UI 撕裂，采用 QVector 缓存数据，并在 paintEvent 中统一绘制。
     */
    QVector<QPointF> pointCloud_;  
};

#endif // SENSORVIEW_H