/**
 * @file View2D.cpp
 * @brief 自动驾驶算法仿真平台 (ADASim) - 全局俯视图 (BEV) 实现
 * @date 2026-04
 * @details
 * 本模块实现了基于第一人称跟随模式的上帝视角渲染。
 * * 【架构与渲染特性】
 * 1. Treadmill 效果：背景网格随车滚动，模拟无限长的行驶路径。
 * 2. 物理一致性：车辆尺寸、轨迹间距与规划偏移量均严格映射至物理单位（米）。
 * 3. 鲁棒性保护：内置时光跳转检测，防止在回放进度条大幅跳动时产生轨迹“连线”污染。
 * 4. 视觉层级 (Z-Order)：背景 -> 网格 -> 历史轨迹 -> 规划路线 -> 障碍物 -> 自车。
 */

#include "View2D.h"
#include "common/Logger.h"
#include <QMouseEvent>
#include <QWheelEvent>
#include <QPainter>
#include <QPen>
#include <QBrush>
#include <cmath>
#include <QPainterPath>

// ============================================================================
// [构造与析构]
// ============================================================================
View2D::View2D(QWidget *parent)
    : QWidget(parent)
    , vehicleX_(0.0)
    , vehicleY_(0.0)
    , vehicleYaw_(0.0)
    , zoom_(5.0)      // 默认缩放倍率：1米对应 5 像素
{
    setMinimumSize(400, 400);
    
    // 设置背景调色板（虽在 paintEvent 中重画，但保留规范属性）
    setAutoFillBackground(true);
    QPalette pal = palette();
    pal.setColor(QPalette::Window, Qt::black);
    setPalette(pal);
}

View2D::~View2D() {}

// ============================================================================
// [数据驱动槽函数]
// ============================================================================

void View2D::setPlannedOffset(double offset)
{
    plannedOffset_ = offset;
    update();
}

void View2D::updateVehiclePosition(double x, double y, double yaw) 
{
    // 【核心架构保护】：时光穿梭检测
    // 如果 X/Y 坐标突变超过 5 米（常见于进度条回溯或算法漂移），立即断开轨迹绘制，防止产生横跨屏幕的连线。
    if (!trajectory_.isEmpty()) {
        QPointF last = trajectory_.last();
        if (std::abs(last.x() - x) > 5.0 || std::abs(last.y() - y) > 5.0) {
            trajectory_.clear();
            LOG_INFO("View2D: 检测到位置突变，已执行历史轨迹重置保护。");
        }
    }
    
    vehicleX_ = x;
    vehicleY_ = y;
    vehicleYaw_ = yaw;
    
    // 更新历史轨迹
    trajectory_.append(QPointF(x, y));
    
    // 【FIFO 队列维护】：严格保持 99 帧轨迹上限，平衡视觉深度与系统性能
    while (trajectory_.size() > 99) {
        trajectory_.pop_front(); 
    }
    
    update();
}

void View2D::updateObstacles(const QVector<QPointF>& obstacles)
{
    obstacles_ = obstacles;
    update();
}

// ============================================================================
// [渲染流水线]
// ============================================================================

void View2D::paintEvent(QPaintEvent* event)
{
    Q_UNUSED(event);
    QPainter painter(this);
    
    // 渲染提示：开启抗锯齿及平滑变换，确保高动态下的直线与曲线不出现毛刺
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setRenderHint(QPainter::SmoothPixmapTransform);
    
    // 1. 绘制科技感渐变背景 (中心发光效果)
    QRadialGradient bgGradient(width()/2, height()/2, width());
    bgGradient.setColorAt(0.0, QColor(15, 22, 38));
    bgGradient.setColorAt(1.0, QColor(5, 8, 17));
    painter.fillRect(rect(), bgGradient);
    
    // 2. 按视觉层级执行绘图子程序
    drawMap(painter);         // 网格层
    drawTrajectory(painter);  // 历史路径层
    drawObstacles(painter);   // 障碍物感知层
    drawPlanning(painter);    // 规控意图层
    drawVehicle(painter);     // 自车本体层
}

void View2D::drawMap(QPainter& painter)
{
    // 暗青色滚动网格
    QPen gridPen(QColor(0, 255, 136, 40), 1);
    painter.setPen(gridPen);
    
    const int gridSize = 50; // 网格间距 (像素)
    // 利用取模运算实现“跑步机”效果，使网格随车平滑滚动
    int offsetX = (int)(vehicleX_ * zoom_) % gridSize;
    int offsetY = (int)(vehicleY_ * zoom_) % gridSize;
    
    for (int x = -offsetX; x < width(); x += gridSize) {
        painter.drawLine(x, 0, x, height());
    }
    for (int y = offsetY; y < height(); y += gridSize) {
        painter.drawLine(0, y, width(), y);
    }
}

void View2D::drawVehicle(QPainter& painter)
{
    painter.save();
    
    // 将坐标系平移至视图 1/4 处（留出前方视野），并根据自车航向角旋转
    int screenX = width() / 4;
    int screenY = height() / 2;
    painter.translate(screenX, screenY);
    painter.rotate(-vehicleYaw_ * 180.0 / M_PI);
    
    // 映射真实物理尺寸 (标准中大型轿车约 4.6m x 2.2m)
    double physicalLength = 4.6;
    double physicalWidth = 2.2;
    int carLength = physicalLength * zoom_;
    int carWidth = physicalWidth * zoom_;
    
    // 1. 绘制危险碰撞包络框 (用于直观评估算法安全冗余)
    painter.setPen(QPen(QColor(255, 50, 50, 100), 1, Qt::DashLine));
    painter.setBrush(QColor(255, 50, 50, 20));
    painter.drawRect(-carLength/2, -carWidth/2, carLength, carWidth);
    
    // 2. 绘制自车主体（渐变科技质感）
    QLinearGradient carGradient(-carLength/2, 0, carLength/2, 0);
    carGradient.setColorAt(0, QColor(220, 220, 230));
    carGradient.setColorAt(1, QColor(160, 170, 180));
    painter.setBrush(carGradient);
    painter.setPen(QPen(QColor(255, 255, 255), 1));
    painter.drawRoundedRect(-carLength/2, -carWidth/2, carLength, carWidth, 4, 4);
    
    // 3. 绘制车头风挡标识 (区分车头方向)
    painter.setBrush(QColor(20, 25, 30));
    painter.setPen(Qt::NoPen);
    painter.drawRect(carLength * 0.1, -carWidth * 0.4, carLength * 0.15, carWidth * 0.8);
    
    painter.restore();
}

void View2D::drawPlanning(QPainter& painter)
{
    painter.save();
    int screenX = width() / 4;
    int screenY = height() / 2;
    painter.translate(screenX, screenY);
    painter.rotate(-vehicleYaw_ * 180.0 / M_PI);
    
    // 定义 Lattice 规划的候选横向采样点
    QVector<double> candidates = {-3.5, -1.75, 0.0, 1.75, 3.5};
    const double planningDistance = 30.0; // 规划预览长度 (米)
    
    for (double offset : candidates) {
        QPainterPath path;
        path.moveTo(0, 0); 
        
        // 采用三阶贝塞尔曲线模拟平滑变道轨迹 (Lattice 简易模型)
        double endY = -offset * zoom_; 
        double c1x = (planningDistance / 3.0) * zoom_;
        double c1y = 0; 
        double c2x = (planningDistance * 2.0 / 3.0) * zoom_;
        double c2y = endY; 
        double endX = planningDistance * zoom_;
        
        path.cubicTo(c1x, c1y, c2x, c2y, endX, endY);
        
        // 渲染区分：最优解用亮色实线，备选解用灰色虚线
        if (std::abs(offset - plannedOffset_) < 0.1) {
            painter.setPen(QPen(QColor(255, 170, 0, 220), 3, Qt::SolidLine));
        } else {
            painter.setPen(QPen(QColor(150, 150, 150, 60), 1, Qt::DashLine));
        }
        painter.drawPath(path);
    }
    painter.restore();
}

void View2D::drawObstacles(QPainter& painter)
{
    painter.setPen(Qt::NoPen);
    
    // 层级 1：全局静态真值 (手动放置的障碍物)
    painter.setBrush(QColor(100, 110, 120, 180)); 
    for (const QPointF& obs : globalUserObstacles_) {
        int x = width()/4 + (obs.x() - vehicleX_) * zoom_;
        int y = height()/2 - (obs.y() - vehicleY_) * zoom_;
        painter.drawEllipse(QPoint(x, y), 6, 6);
    }
    
    // 层级 2：动态感知结果 (雷达检测到的障碍物)
    for (const QPointF& obs : obstacles_) {
        int x = width()/4 + (obs.x() - vehicleX_) * zoom_;
        int y = height()/2 - (obs.y() - vehicleY_) * zoom_;
        
        // 绘制红色发光警告，提醒算法检测成功
        painter.setBrush(QColor(255, 50, 50, 100));
        painter.drawEllipse(QPoint(x, y), 10, 10);
        painter.setBrush(QColor(255, 50, 50, 255));
        painter.drawEllipse(QPoint(x, y), 4, 4);
    }
}

void View2D::drawTrajectory(QPainter& painter)
{
    if (trajectory_.size() < 2) return;
    
    // 历史轨迹渲染：应用 Alpha 渐变，越老的点越透明，营造动态流光感
    for (int i = 1; i < trajectory_.size(); ++i) {
        QPointF p1 = trajectory_[i-1];
        QPointF p2 = trajectory_[i];
        
        int x1 = width()/4 + (p1.x() - vehicleX_) * zoom_;
        int y1 = height()/2 - (p1.y() - vehicleY_) * zoom_;
        int x2 = width()/4 + (p2.x() - vehicleX_) * zoom_;
        int y2 = height()/2 - (p2.y() - vehicleY_) * zoom_;
        
        int alpha = (int)(255.0 * i / trajectory_.size());
        painter.setPen(QPen(QColor(0, 200, 255, alpha), 2, Qt::SolidLine));
        painter.drawLine(x1, y1, x2, y2);
    }
}

// ============================================================================
// [交互事件处理]
// ============================================================================

void View2D::mousePressEvent(QMouseEvent* event)
{
    if (event->button() == Qt::RightButton) {
        // 逆向坐标变换：屏幕像素坐标 -> 全局物理世界坐标 (米)
        double worldX = vehicleX_ + (event->pos().x() - width() / 4.0) / zoom_;
        double worldY = vehicleY_ - (event->pos().y() - height() / 2.0) / zoom_; 
        
        globalUserObstacles_.append(QPointF(worldX, worldY));
        emit userObstacleAdded(worldX, worldY);
        update(); 
    }
    QWidget::mousePressEvent(event);
}

void View2D::wheelEvent(QWheelEvent* event)
{
    // 鼠标滚轮无级缩放
    int delta = event->angleDelta().y();
    if (delta > 0) zoom_ *= 1.1;
    else zoom_ /= 1.1;
    
    // 限制缩放范围：防止无限放大导致精度丢失或无限缩小导致消失
    zoom_ = qBound(0.5, zoom_, 20.0);
    update();
    QWidget::wheelEvent(event);
}

void View2D::mouseMoveEvent(QMouseEvent* event)
{
    // 💡【拓展功能接口】：可在此处实现左键拖拽地图功能 (Panning)
    QWidget::mouseMoveEvent(event);
}