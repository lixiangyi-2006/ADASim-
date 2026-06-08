/**
 * @file View2D.h
 * @brief 自动驾驶算法仿真平台 (ADASim) - 全局二维俯视图模块 (BEV)
 * @date 2026-04
 * @details
 * 本文件定义了负责渲染“上帝视角”物理世界的可视化组件。
 * * 【架构设计与渲染规范】
 * 1. 职责边界：不同于 SensorView（专注于第一人称的纯感知数据），View2D 专注于全局物理坐标系下的宏观呈现，包括自车位姿、规控轨迹、全局地图以及真值对比。
 * 2. 坐标系引擎：底层强烈建议配合 QTransform 矩阵使用，使得所有的 drawXXX 子程序只需关心真实的物理单位（米），坐标偏移与缩放完全交由 GPU/Qt 底层矩阵代劳。
 * 3. 沙盒交互：集成了鼠标与滚轮事件，不仅支持地图的平移缩放，更支持“上帝之手”模式——动态注入沙盒障碍物进行规控算法的闭环测试。
 */

#ifndef VIEW2D_H
#define VIEW2D_H

#include <QWidget>
#include <QPainter>
#include <QVector>
#include <QPolygonF>
#include <QPainterPath>

/**
 * @class View2D
 * @brief 自动驾驶全局俯视交互视图
 */
class View2D : public QWidget
{
    Q_OBJECT
public:
    /**
     * @brief 构造函数，初始化俯视图的 UI 属性及缩放参数
     */
    explicit View2D(QWidget *parent = nullptr);
    ~View2D();
    
public slots:
    // ==========================================
    // [数据驱动槽函数] 接收来自 DataManager/DataLoader 的跨线程数据
    // ==========================================
    
    /**
     * @brief 更新自车的全局物理位姿
     * @param x 物理 X 坐标 (m)
     * @param y 物理 Y 坐标 (m)
     * @param yaw 航向角 (rad)
     */
    void updateVehiclePosition(double x, double y, double yaw);
    
    /**
     * @brief 更新感知算法识别出的障碍物列表
     */
    void updateObstacles(const QVector<QPointF>& obstacles);
    
    /**
     * @brief 接收 Planning 模块下发的规划横向偏移量，用于绘制未来的预测/规划轨迹
     */
    void setPlannedOffset(double offset);

signals:
    // ==========================================
    // [UI 交互信号]
    // ==========================================
    
    /**
     * @brief “上帝之手”触发信号：用户在画面中手动添加了模拟障碍物
     * @param worldX 转换到物理世界的 X 坐标 (m)
     * @param worldY 转换到物理世界的 Y 坐标 (m)
     * * 💡【拓展功能接口】：未来可在此结构中加入 "障碍物类型(人/车/锥桶)" 或 "运动速度(vx, vy)"，
     * 以便实现动态障碍物（Dynamic Obstacles）的沙盒注入测试。
     */
    void userObstacleAdded(double worldX, double worldY);

protected:
    // ==========================================
    // [Qt 核心事件重写]
    // ==========================================
    
    /**
     * @brief 核心重绘事件，调度分层渲染流水线
     */
    void paintEvent(QPaintEvent* event) override;
    
    /**
     * @brief 滚轮事件：控制地图的无极缩放 (Zoom In/Out)
     */
    void wheelEvent(QWheelEvent* event) override;
    
    /**
     * @brief 鼠标按下事件：处理障碍物添加 (右键) 或 拖拽漫游起始 (左键)
     */
    void mousePressEvent(QMouseEvent* event) override;
    
    /**
     * @brief 鼠标移动事件：配合 mousePressEvent 处理拖拽地图漫游
     */
    void mouseMoveEvent(QMouseEvent* event) override;

private:
    // ==========================================
    // [分层渲染流水线子程序]
    // ==========================================
    
    /**
     * @brief 绘制网格、车道线与静态背景地图
     * * 💡【拓展功能接口】：未来接入 OpenDRIVE 或 Lanelet2 高精地图时，在此处解析并绘制路网 Topology。
     */
    void drawMap(QPainter& painter);
    
    /** @brief 绘制自车 (Ego Vehicle) 的 2D 轮廓与航向指示 */
    void drawVehicle(QPainter& painter);
    
    /** @brief 绘制雷达感知到的障碍物包围盒或散点 */
    void drawObstacles(QPainter& painter);
    
    /** @brief 绘制车辆行驶过的历史轨迹 (利用 QPolygonF 批量高速绘制) */
    void drawTrajectory(QPainter& painter);
    
    /** @brief 绘制规划模块 (Planning) 运算出的未来预测行驶意图线 */
    void drawPlanning(QPainter& painter);
    
private:
    // ==========================================
    // [渲染管线内存状态区]
    // * 采用 Modern C++ 规范直接赋初始值，防止野值导致开局画面崩溃
    // ==========================================
    
    // 1. 视口与摄像机控制
    double zoom_ = 20.0;                    ///< 缩放系数 (像素/米，比如 20 代表 1米占 20 像素)
    // TODO: 如果实现了拖拽，还需要加入 cameraOffsetX_ 和 cameraOffsetY_
    
    // 2. 自车物理状态
    double vehicleX_ = 0.0;
    double vehicleY_ = 0.0;
    double vehicleYaw_ = 0.0;
    
    // 3. 感知与规控数据
    QVector<QPointF> obstacles_;            ///< 算法识别到的环境障碍物
    QPolygonF trajectory_;                  ///< 车辆历史轨迹路线
    double plannedOffset_ = 0.0;            ///< 规划下发的横向预期偏移
    
    // 4. 沙盒测试真值注入
    QVector<QPointF> globalUserObstacles_;  ///< 用户利用“上帝之手”手动放置的真值障碍物
};

#endif // VIEW2D_H