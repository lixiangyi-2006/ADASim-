/**
 * @file MainWindow.h
 * @brief 自动驾驶算法仿真平台 (ADASim) - 主控窗口与中枢总线
 * @date 2026-04
 * @details
 * 本文件定义了整个应用程序的主视窗类。它充当“中介者 (Mediator)”，负责协调前端 GUI 渲染、后端数据分发引擎以及网络通信模块。
 * * 【说明】
 * 1. 线程管理：严格遵守 Qt 线程亲和性原则。GUI 必须在主线程运行，而 DataLoader 和 SocketServer 会被平滑地移入 backendThread_ 中，通过跨线程信号槽交互。
 * 2. 指针安全 (Modern C++)：所有成员指针和基础类型均赋初值，杜绝悬挂指针。
 * 3. 闭环物理层：内部集成了简易的运动学/几何物理模型，用于实时响应外部算法下发的方向盘与速度控制指令。
 */

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QThread>
#include <QFrame>
#include <QLabel>
#include <QTextEdit>
#include <QHostAddress>
#include <QComboBox>
#include <QSlider>

// 前置声明 (Forward Declarations)，避免包含过多头文件导致编译缓慢
class View2D;
class SensorView;
class DataLoader;
class SocketServer;
class DataManager;

/**
 * @class MainWindow
 * @brief 系统主窗口与中央控制器
 */
class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    /**
     * @brief 构造函数，初始化 UI 并装载后端子线程
     * @param configPath 配置文件路径
     * @param dataPath 数据资源库路径
     * @param parent 父窗口指针
     */
    explicit MainWindow(const QString& configPath, 
                        const QString& dataPath, 
                        QWidget *parent = nullptr);
    ~MainWindow();
    
signals:
    // ==========================================
    // [跨线程数据分发信号]
    // ==========================================
    
    /**
     * @brief 物理引擎推演完毕信号
     * * 结合了 DataLoader 沙盒坐标与 Python 控制指令后，计算出的真实世界绝对位姿，发回后端供其他模块使用。
     */
    void trueVehiclePositionReady(double x, double y, double yaw);  

private slots:
    // ==========================================
    // [菜单与工具栏交互]
    // ==========================================
    void onOpenScene();
    void onSaveScene();
    void onRecordData();
    void onExit();
    
    // ==========================================
    // [仿真时间轴与运行控制]
    // ==========================================
    void onStartSimulation();
    void onPauseSimulation();
    void onStopSimulation();
    
    // ==========================================
    // [规控指令接收 (闭环控制核心)]
    // ==========================================
    
    /** @brief 接收 Lattice/APF 纯几何平移控制指令 */
    void onLateralControlReceived(double offset); 
    
    /** @brief 接收 MPC/PID 运动学闭环控制指令 (方向盘转角与车速) */
    void onKinematicControlReceived(double steering, double speed); 
    
    // ==========================================
    // [数据流与状态反馈]
    // ==========================================
    void onStatusUpdate(const QString& status);
    void onVehicleDataUpdated(double x, double y, double yaw, double dt);
    void onNetworkDataReceived(const QByteArray& data, const QHostAddress& address);
    void onClientConnected(const QHostAddress& address);
    void onClientDisconnected(const QHostAddress& address);
    void onAlgorithmStatusReceived(const QString& status); // 接收算法端的运行状态 (如：Lattice 避障中)
    
private:
    // ==========================================
    // [UI 构建子程序]
    // ==========================================
    void setupUI();
    void setupMenuBar();
    void setupToolBar();
    void setupStatusBar();
    void setupConnections();
    
    void createInfoCard(QLayout* layout, const QString& title, const QString& value, const QString& objectName);
    QFrame* createAlgoPanel();
    
    // ==========================================
    // [后端生命周期管理]
    // ==========================================
    void startBackend();
    void stopBackend();
    
protected:
    /** @brief 重写窗口关闭事件，确保退出前安全中止子线程与释放 Socket */
    void closeEvent(QCloseEvent* event) override;
    
private:
    // ==========================================
    // [UI 视图组件引用] 统一初始化为 nullptr
    // ==========================================
    View2D* view2D_ = nullptr;                  ///< BEV 上帝视角渲染器
    SensorView* sensorView_ = nullptr;          ///< 雷达点云感知渲染器
    QTextEdit* consoleText_ = nullptr;          ///< 底部控制台输出窗口
    
    QLabel* speedValue_ = nullptr;              ///< 仪表盘：速度
    QLabel* distanceValue_ = nullptr;           ///< 仪表盘：里程
    QLabel* fpsValue_ = nullptr;                ///< 仪表盘：帧率
    QLabel* algoValue_ = nullptr;               ///< 仪表盘：算法模式
    QLabel* connLabel_ = nullptr;               ///< 状态栏：网络连接状态灯
    
    QComboBox* algoCombo_ = nullptr;            ///< 感知算法选择下拉框
    QComboBox* planAlgoCombo_ = nullptr;        ///< 规控算法选择下拉框
    QLabel* algoStatusLabel_ = nullptr;         ///< Python 传回的动态算法状态提示
    
    QSlider* timeSlider_ = nullptr;             ///< 行车记录仪时间轴
    QLabel* timeLabel_ = nullptr;               ///< 时间轴进度标签 ("当前/总帧数")

    // ==========================================
    // [后端服务与线程句柄]
    // ==========================================
    QThread* backendThread_ = nullptr;          ///< 后端高频数据处理专用子线程
    DataLoader* dataLoader_ = nullptr;          ///< 沙盒数据泵引擎
    SocketServer* socketServer_ = nullptr;      ///< TCP/UDP 网络通信网关
    DataManager* dataManager_ = nullptr;        ///< 结构化数据解析与组装中心
    
    QString configPath_;                        ///< 启动配置路径
    QString dataPath_;                          ///< 数据资源路径
    
    // ==========================================
    // [仿真宏观统计状态]
    // ==========================================
    int activeClients_ = 0;                     ///< 活跃的算法端连接数
    double totalDistance_ = 0.0;                ///< 累计行驶里程
    double lastX_ = 0.0;                        ///< 上一帧全局 X 坐标 (用于计算里程)
    double lastY_ = 0.0;                        ///< 上一帧全局 Y 坐标
    
    // ==========================================
    // [内置简易物理引擎状态区]
    // 💡【拓展功能接口】：随着自动驾驶模型的复杂化，此处关于 offset, speed, yaw 
    // 的计算逻辑应剥离成一个独立的 VehicleDynamics 类，支持轮胎侧滑角等高级计算。
    // ==========================================
    
    // 1. 纯几何平移控制 (Lattice/APF 算法使用)
    double targetLateralOffset_ = 0.0;          ///< 算法下发的期望横向偏移量
    double currentLateralOffset_ = 0.0;         ///< 动画平滑插值后的当前横向偏移量
    
    // 2. 运动学闭环控制 (MPC/PID 算法使用)
    double currentSteering_ = 0.0;              ///< 当前方向盘转角 (弧度)
    double currentSpeed_ = 10.0;                ///< 车辆巡航速度 (默认 10m/s，约 36km/h)
    const double WHEELBASE = 2.8;               ///< 车辆轴距参数 (米)
    
    double lastEgoY_ = 0.0;                     ///< 闭环物理推演下的真实横向位置 Y
    double lastEgoYaw_ = 0.0;                   ///< 闭环物理推演下的真实车头朝向 Yaw
};

#endif // MAINWINDOW_H
