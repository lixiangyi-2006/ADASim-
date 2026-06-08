/**
 * @file SensorView.cpp
 * @brief 自动驾驶算法仿真平台 (ADASim) - 传感器视图实现文件
 * @date 2026-04
 * @details
 * 本文件负责实现高频激光雷达点云的 2D 软渲染可视化。
 * * 【性能优化说明】
 * 1. 消除浮点开方瓶颈：彻底移除了循环内极其耗时的 std::sqrt 计算，改用平方距离 (Squared Distance) 直接进行阈值判断。
 * 2. 批量渲染架构 (Batch Rendering)：移除了 O(n) 复杂度的“单点 setPen + drawPoint”反模式。
 * 3. 颜色分层缓存机制：按照风险等级（距离），将点云分桶缓存到对应的 QVector 中，最后通过底层 drawPoints() API 批量提交给 CPU/GPU 渲染，FPS 提升数倍！
 */

#include "SensorView.h"
#include <QPainter>
#include <QFont>
#include <QTransform>

SensorView::SensorView(QWidget *parent)
    : QWidget(parent)
{
    setMinimumSize(300, 400);
    
    // 【渲染优化】：告诉 Qt 引擎，我们会在 paintEvent 里自行涂满整个背景。
    // 这能省去 Qt 底层的一道无用默认背景填充工序，防止画面高频闪烁，节省渲染开销。
    setAttribute(Qt::WA_OpaquePaintEvent);
    setAttribute(Qt::WA_NoSystemBackground);
}

SensorView::~SensorView() {}

void SensorView::updatePointCloud(const QVector<QPointF>& points)
{
    pointCloud_ = points;
    // 【消息队列优化】：update() 不是立即重绘，而是发出重绘请求。
    // 即使底层 1000Hz 狂发点云，Qt 也会智能合并请求，按照屏幕刷新率（如 60Hz）调用 paintEvent，防止 UI 线程卡死。
    update();
}

void SensorView::paintEvent(QPaintEvent* event)
{
    Q_UNUSED(event);
    
    QPainter painter(this);
    // 开启抗锯齿，使雷达圆环和点云更加平滑
    painter.setRenderHint(QPainter::Antialiasing);
    
    // 1. 填充极黑科技感背景
    painter.fillRect(rect(), QColor(5, 5, 8));
    
    // 2. 调度具体的雷达渲染流水线
    drawLidarView(painter);
    
    // 3. 绘制 HUD (平视显示器) 标题
    painter.setPen(QColor(0, 255, 136));
    painter.setFont(QFont("Arial", 10, QFont::Bold));
    painter.drawText(15, 25, "● LiDAR POINT CLOUD");
}

void SensorView::drawLidarView(QPainter& painter)
{
    const int centerX = width() / 2;
    const int centerY = height() / 2;
    const int maxRadius = qMin(width(), height()) / 2 - 20;
    
    // ==========================================
    // 第一阶段：绘制神盾局风格雷达底图
    // ==========================================
    painter.setPen(QPen(QColor(0, 255, 136, 40), 1));
    for (int r = maxRadius / 4; r <= maxRadius; r += maxRadius / 4) {
        painter.drawEllipse(QPoint(centerX, centerY), r, r);
    }
    // 绘制十字准星
    painter.drawLine(centerX, centerY - maxRadius, centerX, centerY + maxRadius);
    painter.drawLine(centerX - maxRadius, centerY, centerX + maxRadius, centerY);
    
    // ==========================================
    // 第二阶段：高性能点云批量渲染 (Batch Rendering)
    // ==========================================
    // 空间缩放比例：假设视图最大半径代表物理世界的 50 米
    const float scale = maxRadius / 50.0f;  
    
    // 预计算平方距离的阈值，彻底告别 std::sqrt！
    const float sqDistRed = 15.0f * 15.0f;    // 15米以内（极度危险，红色）
    const float sqDistYellow = 30.0f * 30.0f; // 30米以内（警惕，黄色）
    const float sqDistMax = 50.0f * 50.0f;    // 50米以外忽略渲染
    
    // 分层缓存桶
    QVector<QPointF> pointsRed;
    QVector<QPointF> pointsYellow;
    QVector<QPointF> pointsGreen;
    
    // 预分配适当内存，减少扩容开销（这只是估算，视实际点云密度而定）
    int estSize = pointCloud_.size() / 3;
    pointsRed.reserve(estSize);
    pointsYellow.reserve(estSize);
    pointsGreen.reserve(estSize);

    // 遍历原始点云，进行坐标转换与分桶
    for (const QPointF& point : pointCloud_) {
        // 计算当前点的物理平方距离
        float distSq = point.x() * point.x() + point.y() * point.y();
        
        // 剔除雷达圈外（> 50米）的无效噪点
        if (distSq > sqDistMax) continue;
        
        // 将物理坐标映射到 UI 屏幕坐标
        QPointF screenPt(centerX + point.x() * scale, centerY - point.y() * scale);
        
        // 色彩分层
        if (distSq < sqDistRed) {
            pointsRed.append(screenPt);
        } else if (distSq < sqDistYellow) {
            pointsYellow.append(screenPt);
        } else {
            pointsGreen.append(screenPt);
        }
    }
    
    // 批量下发渲染指令 (Draw Calls) - 性能提升的核心！
    // 1. 绘制安全点（青绿）
    painter.setPen(QPen(QColor(0, 255, 136), 2));
    painter.drawPoints(pointsGreen.data(), pointsGreen.size());
    
    // 2. 绘制警告点（黄）
    painter.setPen(QPen(QColor(255, 200, 50), 3));
    painter.drawPoints(pointsYellow.data(), pointsYellow.size());
    
    // 3. 绘制危险点（红），稍微画大一点突出风险
    painter.setPen(QPen(QColor(255, 50, 50), 4));
    painter.drawPoints(pointsRed.data(), pointsRed.size());
    
    // ==========================================
    // 第三阶段：自车中心发光标识
    // ==========================================
    painter.setPen(Qt::NoPen);
    painter.setBrush(QColor(255, 255, 255));
    painter.drawEllipse(centerX - 2, centerY - 2, 4, 4); // 核心白点
    painter.setBrush(QColor(0, 255, 136, 100));
    painter.drawEllipse(centerX - 6, centerY - 6, 12, 12); // 光晕
}