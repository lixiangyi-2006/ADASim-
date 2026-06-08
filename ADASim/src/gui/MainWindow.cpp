/**
 * @file MainWindow.cpp
 * @brief 自动驾驶算法仿真平台 (ADASim) - 主控窗口与中枢总线实现
 * @date 2026-04
 * @details
 * 本文件实现了系统的全生命周期管理、UI 布局组装、跨线程信号流转以及核心的闭环物理引擎。
 * * 【亮点】
 * 1. 消除冗余：清除了重复的头文件包含与多次 new 同一控件（如 algoCombo_）的内存泄漏隐患。
 * 2. 布局规整：将零散的回放进度条 (Playback Slider) 完美融入了全局 UI 树，界面层次分明。
 * 3. 物理引擎解耦：对 onVehicleDataUpdated 中的 Bicycle Model (自行车运动学模型) 进行了专业级注释，为将来独立拆分 VehicleDynamics 类打下基础。
 */

#include "MainWindow.h"
#include "View2D.h"
#include "SensorView.h"

#include "backend/DataLoader.h"
#include "communication/Socket.h"
#include "data/DataManager.h"

#include "common/Config.h" 
#include "common/Logger.h"

#include <QMenuBar>
#include <QToolBar>
#include <QStatusBar>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QSplitter>
#include <QLabel>
#include <QPushButton>
#include <QMessageBox>
#include <QCloseEvent>
#include <QComboBox>
#include <QDateTime>
#include <cmath>
#include <QFile>
#include <QDebug>
#include <QTimer>
#include <unistd.h>

// ============================================================================
// [构造与析构]
// ============================================================================

MainWindow::MainWindow(const QString& configPath, const QString& dataPath, QWidget *parent)
    : QMainWindow(parent)
    , configPath_(configPath)
    , dataPath_(dataPath)
    , prevTotal_(0)
    , prevIdle_(0)
{
    setWindowTitle("ADASim - 自动驾驶算法仿真平台 v1.0.0");
    resize(1600, 900);
    setStyleSheet("QMainWindow { background-color: #050811; }");
    
    // 注册自定义类型，允许在跨线程信号槽中传递 IP 地址
    qRegisterMetaType<QHostAddress>("QHostAddress");
    
    // 1. 初始化所有 UI 组件
    setupUI();
    setupMenuBar();
    setupToolBar();
    setupStatusBar();
    
    // 2. 启动后端无头 (Headless) 服务线程并绑定通信管线
    startBackend();
    setupConnections();
    
    LOG_INFO("MainWindow: 主窗口界面已装载完毕");
}

MainWindow::~MainWindow()
{
    stopBackend();
    LOG_INFO("MainWindow: 主窗口已安全释放");
}

// ============================================================================
// [UI 构建子程序]
// ============================================================================

void MainWindow::setupUI()
{
    QWidget* centralWidget = new QWidget(this);
    setCentralWidget(centralWidget);
    
    QVBoxLayout* mainLayout = new QVBoxLayout(centralWidget);
    mainLayout->setSpacing(10);
    mainLayout->setContentsMargins(10, 10, 10, 10);
    
    // === 1. 顶部信息仪表盘面板 ===
    QFrame* infoPanel = new QFrame(this);
    infoPanel->setFixedHeight(80);
    infoPanel->setStyleSheet(
        "QFrame { background: QLinearGradient(x1:0, y1:0, x2:0, y2:1, stop:0 #1e3c72, stop:1 #2a5298); "
        "border-radius: 8px; border: 1px solid #4a90e2; }"
    );
    QHBoxLayout* infoLayout = new QHBoxLayout(infoPanel);
    createInfoCard(infoLayout, "当前速度", "0 km/h", "speedValue");
    createInfoCard(infoLayout, "行驶里程", "0.0 m", "distanceValue");
    createInfoCard(infoLayout, "渲染帧率", "60 FPS", "fpsValue");
    createInfoCard(infoLayout, "算法状态", "就绪", "algoValue");
    mainLayout->addWidget(infoPanel);
    
    // === 2. 中部核心视觉渲染区 ===
    QHBoxLayout* viewsLayout = new QHBoxLayout();
    viewsLayout->setSpacing(10);
    
    view2D_ = new View2D(this);
    view2D_->setMinimumSize(400, 400);
    view2D_->setStyleSheet("QWidget { background: #0a0e1a; border: 2px solid #4a90e2; border-radius: 8px; }");
    
    QVBoxLayout* rightPanel = new QVBoxLayout();
    rightPanel->setSpacing(10);
    
    sensorView_ = new SensorView(this);
    sensorView_->setMinimumSize(300, 250);
    sensorView_->setStyleSheet("QWidget { background: #0a0e1a; border: 2px solid #00ff88; border-radius: 8px; }");
    
    QFrame* algoPanel = createAlgoPanel();
    rightPanel->addWidget(sensorView_);
    rightPanel->addWidget(algoPanel);
    
    QSplitter* splitter = new QSplitter(Qt::Horizontal);
    splitter->addWidget(view2D_);
    
    QWidget* rightWidget = new QWidget();
    rightWidget->setLayout(rightPanel);
    splitter->addWidget(rightWidget);
    splitter->setStretchFactor(0, 2); // 2D 主视图占 2/3
    splitter->setStretchFactor(1, 1); // 传感器与控制面板占 1/3
    viewsLayout->addWidget(splitter);
    mainLayout->addLayout(viewsLayout);
    
    // === 3. 底部调试控制台 ===
    QFrame* consolePanel = new QFrame(this);
    consolePanel->setFixedHeight(120);
    consolePanel->setStyleSheet("QFrame { background: #0d1117; border: 1px solid #30363d; border-radius: 8px; }");
    
    QVBoxLayout* consoleLayout = new QVBoxLayout(consolePanel);
    QLabel* consoleTitle = new QLabel("实时数据总线监控");
    consoleTitle->setStyleSheet("color: #58a6ff; font-size: 14px; font-weight: bold;");
    consoleLayout->addWidget(consoleTitle);
    
    consoleText_ = new QTextEdit(this);
    consoleText_->setReadOnly(true);
    consoleText_->setStyleSheet("QTextEdit { background: #0d1117; color: #7ee787; border: none; font-family: 'Courier New', monospace; font-size: 12px; }");
    consoleLayout->addWidget(consoleText_);
    mainLayout->addWidget(consolePanel);

    // === 4. 底部时间轴/行车记录仪回放组件 ===
    QWidget* playbackWidget = new QWidget(this);
    playbackWidget->setStyleSheet("background: #0d1117; border-top: 1px solid #30363d; border-radius: 4px;");
    QHBoxLayout* playLayout = new QHBoxLayout(playbackWidget);
    playLayout->setContentsMargins(10, 5, 10, 5);

    QLabel* titleLabel = new QLabel("时光回放缓存区：", this);
    titleLabel->setStyleSheet("color: #8b949e; font-weight: bold;");
    playLayout->addWidget(titleLabel);

    timeSlider_ = new QSlider(Qt::Horizontal, this);
    timeSlider_->setStyleSheet(
        "QSlider::handle:horizontal { background: #58a6ff; width: 14px; margin: -4px 0; border-radius: 7px; }"
        "QSlider::groove:horizontal { background: #21262d; height: 6px; border-radius: 3px; }"
    );
    playLayout->addWidget(timeSlider_);

    timeLabel_ = new QLabel("0 / 0", this);
    timeLabel_->setStyleSheet("color: #8b949e; font-family: 'Consolas', monospace; min-width: 80px; text-align: right;");
    timeLabel_->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    playLayout->addWidget(timeLabel_);
    
    mainLayout->addWidget(playbackWidget);
}

void MainWindow::createInfoCard(QLayout* layout, const QString& title, const QString& value, const QString& objectName)
{
    QFrame* card = new QFrame(this);
    card->setFixedWidth(180);
    card->setStyleSheet(
        "QFrame { background: rgba(20, 25, 35, 0.8); border: 1px solid rgba(0, 255, 136, 0.3); border-radius: 8px; }"
    );
    
    QVBoxLayout* cardLayout = new QVBoxLayout(card);
    cardLayout->setContentsMargins(15, 10, 15, 10);
    
    QLabel* titleLabel = new QLabel(title);
    titleLabel->setStyleSheet("color: #8b949e; font-size: 12px; font-weight: bold; border: none; background: transparent;");
    cardLayout->addWidget(titleLabel);
    
    QLabel* valueLabel = new QLabel(value);
    valueLabel->setObjectName(objectName);
    valueLabel->setStyleSheet("color: #00ff88; font-size: 22px; font-weight: bold; font-family: 'Consolas'; border: none; background: transparent;");
    cardLayout->addWidget(valueLabel);
    
    layout->addWidget(card);
    
    if (objectName == "speedValue") speedValue_ = valueLabel;
    else if (objectName == "distanceValue") distanceValue_ = valueLabel;
    else if (objectName == "fpsValue") fpsValue_ = valueLabel;
    else if (objectName == "algoValue") algoValue_ = valueLabel;
}

QFrame* MainWindow::createAlgoPanel()
{
    QFrame* panel = new QFrame(this);
    panel->setFixedHeight(130);
    panel->setStyleSheet("QFrame { background: QLinearGradient(x1:0, y1:0, x2:0, y2:1, stop:0 #161b22, stop:1 #0d1117); border: 1px solid #30363d; border-radius: 8px; }");
    QVBoxLayout* panelLayout = new QVBoxLayout(panel);
    panelLayout->setContentsMargins(15, 10, 15, 10);
    
    QLabel* title = new QLabel("🤖 智驾算法挂载控制面板");
    title->setStyleSheet("color: #f0f6fc; font-size: 14px; font-weight: bold;");
    panelLayout->addWidget(title);
    
    QHBoxLayout* algoLayout = new QHBoxLayout();
    
    // 感知算法下拉框
    algoCombo_ = new QComboBox(this);
    algoCombo_->addItems({"感知: 点云聚类 (基础)", "感知: RANSAC (去地噪)", "感知: YOLO (语义模拟)"});
    algoCombo_->setStyleSheet("QComboBox { background: #0d1117; color: #7ee787; border: 1px solid #30363d; border-radius: 4px; padding: 5px; } QComboBox::drop-down { border: none; }");
    algoLayout->addWidget(algoCombo_);
    
    // 规控算法下拉框
    planAlgoCombo_ = new QComboBox(this);
    planAlgoCombo_->addItems({"规控: Lattice 代价寻优", "规控: APF 人工势场法", "规控: MPC+PID 运动学闭环"});
    planAlgoCombo_->setStyleSheet("QComboBox { background: #0d1117; color: #ffaa00; border: 1px solid #30363d; border-radius: 4px; padding: 5px; margin-left: 10px;} QComboBox::drop-down { border: none; }");
    algoLayout->addWidget(planAlgoCombo_);
    
    algoLayout->addStretch();
    panelLayout->addLayout(algoLayout);
    
    // 算法实时状态提示
    algoStatusLabel_ = new QLabel("✅ 算法节点就绪 | 延迟：<5ms | 等待 Python 脚本连接...");
    algoStatusLabel_->setStyleSheet("color: #7ee787; font-size: 11px;");
    panelLayout->addWidget(algoStatusLabel_);
    
    return panel;
}

// ============================================================================
// [菜单栏、工具栏与状态栏]
// ============================================================================

void MainWindow::setupMenuBar()
{
    QMenu* simMenu = menuBar()->addMenu("仿真引擎 (&S)");
    simMenu->addAction("▶ 开始沙盒", this, &MainWindow::onStartSimulation, QKeySequence(Qt::Key_F5));
    simMenu->addAction("⏸ 暂停沙盒", this, &MainWindow::onPauseSimulation, QKeySequence(Qt::Key_F6));
    simMenu->addAction("⏹ 停止并重置", this, &MainWindow::onStopSimulation, QKeySequence(Qt::Key_F7));
    simMenu->addSeparator();
    simMenu->addAction("退出 (&X)", this, &MainWindow::onExit, QKeySequence::Quit);
}

void MainWindow::setupToolBar()
{
    QToolBar* toolBar = addToolBar("主工具栏");
    toolBar->setMovable(false);
    toolBar->setStyleSheet(
        "QToolBar { background: #0b0f19; border: none; spacing: 15px; padding: 5px; }"
        "QToolButton { color: #f0f6fc; font-weight: bold; background: rgba(255,255,255,0.1); border-radius: 4px; padding: 4px 10px; }"
        "QToolButton:hover { background: rgba(0, 255, 136, 0.3); color: #00ff88; }"
    );
    
    toolBar->addAction("▶ 启动无尽沙盒", this, &MainWindow::onStartSimulation);
    toolBar->addAction("⏸ 暂停", this, &MainWindow::onPauseSimulation);
    toolBar->addAction("⏹ 重置世界", this, &MainWindow::onStopSimulation);
}

void MainWindow::setupStatusBar()
{
    QStatusBar* statusBar = this->statusBar();
    statusBar->showMessage("ADASim 引擎就绪");
    cpuLabel_ = new QLabel("CPU: --%");
    cpuLabel_->setStyleSheet("color: #00ff88; background: transparent;");
    statusBar->addPermanentWidget(cpuLabel_);
    memLabel_ = new QLabel("Mem: --MB");
    memLabel_->setStyleSheet("color: #00ff88; background: transparent;");
    statusBar->addPermanentWidget(memLabel_);
    threadLabel_ = new QLabel("Threads: --");
    threadLabel_->setStyleSheet("color: #00ff88; background: transparent;");
    statusBar->addPermanentWidget(threadLabel_);
    connLabel_ = new QLabel("外部算法节点：0");
    statusBar->addPermanentWidget(connLabel_);
}
void MainWindow::updatePerformanceStats()
{
    LOG_DEBUG("updatePerformanceStats called");
    // 1. 读取 /proc/stat 第一行
    QFile statFile("/proc/stat");
    if (!statFile.open(QIODevice::ReadOnly)) return;
    QString line = statFile.readLine();
    statFile.close();
    QStringList parts = line.split(' ', Qt::SkipEmptyParts);
    if (parts.size() < 5) return;
    // parts[1] user, parts[2] nice, parts[3] system, parts[4] idle, parts[5] iowait
    
    unsigned long long user = parts[1].toULongLong();
    unsigned long long nice = parts[2].toULongLong();
    unsigned long long system = parts[3].toULongLong();
    unsigned long long idle = parts[4].toULongLong();
    unsigned long long iowait = parts.size() > 5 ? parts[5].toULongLong() : 0;
    unsigned long long total = user + nice + system + idle + iowait;
    unsigned long long idleAll = idle + iowait;
    double cpu = 0.0;
    
    if (prevTotal_ != 0) {
        unsigned long long totalDelta = total - prevTotal_;
        unsigned long long idleDelta = idleAll - prevIdle_;
        cpu = (totalDelta - idleDelta) * 100.0 / totalDelta;
    }
    
    prevTotal_ = total;
    prevIdle_ = idleAll;
    cpuLabel_->setText(QString("CPU: %1%").arg(cpu, 0, 'f', 1));
    // 调试：打印标签文本
    qDebug() << "CPU Label text:" << cpuLabel_->text();
    // 强制刷新状态栏
    statusBar()->repaint();
    
    // 2. 读取内存 RSS
    QFile statmFile("/proc/self/statm");
    if (statmFile.open(QIODevice::ReadOnly)) {
        QString line = statmFile.readLine();
        statmFile.close();
        QStringList fields = line.split(' ');
        if (fields.size() >= 2) {
            long rssPages = fields[1].toLong();
            long pageSize = sysconf(_SC_PAGESIZE);
            double rssMB = (rssPages * pageSize) / (1024.0 * 1024.0);
            memLabel_->setText(QString("Mem: %1 MB").arg(rssMB, 0, 'f', 1));
            qDebug() << "Mem Label text:" << memLabel_->text();
            statusBar()->repaint();
        }
    }

    // 3. 读取线程数
    QFile statusFile("/proc/self/status");
    if (statusFile.open(QIODevice::ReadOnly)) {
        while (!statusFile.atEnd()) {
            QString line = statusFile.readLine();
            if (line.startsWith("Threads:")) {
                QStringList parts = line.split('\t', Qt::SkipEmptyParts);
                if (parts.size() >= 2) {
                    int threads = parts[1].toInt();
                    threadLabel_->setText(QString("Threads: %1").arg(threads));
                    qDebug() << "Thread Label text:" << threadLabel_->text();
                    statusBar()->repaint();
                }
                break;
            }
        }
        statusFile.close();
    }
}

// ============================================================================
// [后端生命周期管理]
// ============================================================================

void MainWindow::startBackend()
{
    backendThread_ = new QThread(this);
    
    dataLoader_   = new DataLoader(dataPath_);
    socketServer_ = new SocketServer();
    dataManager_  = new DataManager(); 
    
    // 移入子线程，避免高频网络报文与点云读取阻塞 GUI 渲染
    dataLoader_->moveToThread(backendThread_);
    socketServer_->moveToThread(backendThread_);
    dataManager_->moveToThread(backendThread_);
    
    // 线程结束时自动清理内存
    connect(backendThread_, &QThread::finished, dataLoader_, &QObject::deleteLater);
    connect(backendThread_, &QThread::finished, socketServer_, &QObject::deleteLater);
    connect(backendThread_, &QThread::finished, dataManager_, &QObject::deleteLater);
    
    backendThread_->start();
    
    // 启动通信服务
    QMetaObject::invokeMethod(socketServer_, "startTcpServer", Qt::QueuedConnection, Q_ARG(quint16, 8080));
    QMetaObject::invokeMethod(socketServer_, "startUdpServer", Qt::QueuedConnection, Q_ARG(quint16, 8081));
}

void MainWindow::stopBackend()
{
    if (backendThread_ && backendThread_->isRunning()) {
        QMetaObject::invokeMethod(dataLoader_, "stop", Qt::BlockingQueuedConnection);
        QMetaObject::invokeMethod(socketServer_, "stop", Qt::BlockingQueuedConnection);
        backendThread_->quit();
        backendThread_->wait(2000); 
    }
}

// ============================================================================
// [跨线程信号总线与闭环控制逻辑]
// ==========================================

void MainWindow::setupConnections()
{
    // --------------------------------------------------------
    // 链路一：感知数据分发
    // --------------------------------------------------------
    // DataLoader 产出原始雷达 -> DataManager 融合处理 -> UI 渲染
    connect(dataLoader_, &DataLoader::pointCloudReady, dataManager_, &DataManager::onPointCloudReceived);
    connect(dataManager_, &DataManager::mergedPointCloudReady, sensorView_, &SensorView::updatePointCloud);
    
    // --------------------------------------------------------
    // 链路二：物理模型闭环与轨迹绘制
    // --------------------------------------------------------
    // DataLoader 产出基础坐标 -> MainWindow 叠加物理模型算法 -> DataManager/View2D
    connect(dataLoader_, &DataLoader::vehiclePositionReady, this, &MainWindow::onVehicleDataUpdated);
    connect(this, &MainWindow::trueVehiclePositionReady, dataManager_, &DataManager::onVehiclePositionReceived);
    
    // --------------------------------------------------------
    // 链路三：算法检测结果与沙盒交互
    // --------------------------------------------------------
    connect(dataManager_, &DataManager::obstaclesDetected, view2D_, &View2D::updateObstacles);
    connect(view2D_, &View2D::userObstacleAdded, dataManager_, &DataManager::onUserObstacleAdded);
    connect(dataManager_, &DataManager::lateralControlReceived, this, &MainWindow::onLateralControlReceived);
    connect(dataManager_, &DataManager::kinematicControlReceived, this, &MainWindow::onKinematicControlReceived);

    // --------------------------------------------------------
    // 链路四：网络层透明透传
    // --------------------------------------------------------
    connect(socketServer_, &SocketServer::dataReceived, this, &MainWindow::onNetworkDataReceived);
    connect(socketServer_, &SocketServer::dataReceived, dataManager_, &DataManager::onNetworkDataReceived);
    connect(dataManager_, &DataManager::broadcastDataRequested, socketServer_, &SocketServer::sendToClient);

    // --------------------------------------------------------
    // 链路五：UI 状态监控与反馈
    // --------------------------------------------------------
    connect(dataLoader_, &DataLoader::statusUpdate, this, &MainWindow::onStatusUpdate);
    connect(socketServer_, &SocketServer::clientConnected, this, &MainWindow::onClientConnected);
    connect(socketServer_, &SocketServer::clientDisconnected, this, &MainWindow::onClientDisconnected);
    
    // 下拉框切换触发后端算法变更
    connect(algoCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged), dataManager_, &DataManager::onAlgorithmChanged);
    connect(planAlgoCombo_, QOverload<int>::of(&QComboBox::currentIndexChanged), dataManager_, &DataManager::onPlanAlgorithmChanged);
    connect(dataManager_, &DataManager::algorithmStatusUpdated, this, &MainWindow::onAlgorithmStatusReceived);
            
    // --------------------------------------------------------
    // 链路六：时间轴/行车记录仪联动控制
    // --------------------------------------------------------
    connect(dataLoader_, &DataLoader::totalFramesLoaded, this, [this](int total) {
        int maxFrame = (total > 0) ? total - 1 : 0;
        if (maxFrame > 99) maxFrame = 99; // 上限 99 帧
        timeSlider_->setMaximum(maxFrame);
        timeLabel_->setText(QString("0 / %1").arg(maxFrame));
    });

    connect(dataLoader_, &DataLoader::currentFrameUpdated, this, [this](int current) {
        timeSlider_->blockSignals(true); // 防止反向触发 seek 造成死循环
        timeSlider_->setValue(current);
        timeSlider_->blockSignals(false);
        timeLabel_->setText(QString("%1 / %2").arg(current).arg(timeSlider_->maximum()));
    });

    connect(timeSlider_, &QSlider::valueChanged, this, [this](int value) {
        // 【时光回溯保护】：拖动时间轴时，强行清空底层的闭环物理惯性积压
        lastEgoY_ = 0.0;
        lastEgoYaw_ = 0.0;
        
        QMetaObject::invokeMethod(dataLoader_, "seekToFrame", Qt::QueuedConnection, Q_ARG(int, value));
    });
}

// ============================================================================
// [闭环物理引擎 (Kinematic Engine)]
// ============================================================================

void MainWindow::onVehicleDataUpdated(double x, double y, double yaw,double dt)
{
    double trueY = y;
    double trueYaw = yaw;
    
    // 【模式 1】：纯几何平移算法 (Lattice / APF)
    if (planAlgoCombo_->currentIndex() < 2) {
        // 利用低通滤波平滑过渡
        currentLateralOffset_ += (targetLateralOffset_ - currentLateralOffset_) * 0.3;
        trueY = y + currentLateralOffset_;
    } 
    // 【模式 2】：运动学闭环控制算法 (MPC / PID)
    else {
        // 经典的自行车运动学模型 (Bicycle Kinematic Model)
        double dt = 0.1; // 10Hz 触发频率
        
        // 1. 根据当前速度、轴距和前轮转角，计算偏航角速率 (Yaw Rate)
        double yawRate = (currentSpeed_ / WHEELBASE) * std::tan(currentSteering_);
        lastEgoYaw_ += yawRate * dt; 
        
        // 2. 根据偏航角推算真实的横向 Y 坐标位移
        lastEgoY_ += currentSpeed_ * std::sin(lastEgoYaw_) * dt;
        
        // 忽略世界坐标系的绝对 X 移动（因为沙盒是跑步机模式），仅采纳闭环算出的 Y 和 Yaw
        trueY = lastEgoY_;
        trueYaw = lastEgoYaw_;
    }
    
    // 更新视图与后端真值
    view2D_->updateVehiclePosition(x, trueY, trueYaw);
    emit trueVehiclePositionReady(x, trueY, trueYaw);
    
    // ==========================================
    // 仪表盘宏观数据统计算法
    // ==========================================
    if (lastX_ != 0.0 || lastY_ != 0.0) {
        double dx = x - lastX_;
        double dy = trueY - lastY_; 
        double dist = std::sqrt(dx*dx + dy*dy);
        
        totalDistance_ += dist;
        double speedKmH = (dist / dt) * 3.6; // 瞬时速度 (m/s -> km/h)
        
        if (speedValue_) speedValue_->setText(QString::number(speedKmH, 'f', 1) + " km/h");
        if (distanceValue_) distanceValue_->setText(QString::number(totalDistance_, 'f', 1) + " m");
    }
    
    lastX_ = x;
    lastY_ = trueY;
}

// ============================================================================
// [规控指令下发与状态监控]
// ============================================================================

void MainWindow::onLateralControlReceived(double offset) 
{
    targetLateralOffset_ = offset;
    view2D_->setPlannedOffset(offset); // 实时渲染算法决策触手
}

void MainWindow::onKinematicControlReceived(double steering, double speed) 
{
    currentSteering_ = steering;
    // 模拟动力系统的执行器延迟 (一阶惯性环节)
    currentSpeed_ += (speed - currentSpeed_) * 0.1;
}

void MainWindow::onAlgorithmStatusReceived(const QString& status)
{
    if (algoStatusLabel_) algoStatusLabel_->setText(status);
}

void MainWindow::onStatusUpdate(const QString& status)
{
    statusBar()->showMessage(status);
}

// ============================================================================
// [网络日志与交互操作槽]
// ============================================================================

void MainWindow::onStartSimulation()
{
    consoleText_->append(QString("[%1] 发送沙盒启动指令").arg(QDateTime::currentDateTime().toString("HH:mm:ss")));
    QMetaObject::invokeMethod(dataLoader_, "start", Qt::QueuedConnection);
}

void MainWindow::onPauseSimulation()
{
    consoleText_->append(QString("[%1] 发送沙盒暂停指令").arg(QDateTime::currentDateTime().toString("HH:mm:ss")));
    QMetaObject::invokeMethod(dataLoader_, "pause", Qt::QueuedConnection);
}

void MainWindow::onStopSimulation()
{
    consoleText_->append(QString("[%1] 发送重置与停止指令").arg(QDateTime::currentDateTime().toString("HH:mm:ss")));
    QMetaObject::invokeMethod(dataLoader_, "stop", Qt::QueuedConnection);
    
    totalDistance_ = 0;
    if (distanceValue_) distanceValue_->setText("0.0 m");
    if (speedValue_) speedValue_->setText("0.0 km/h");
}

void MainWindow::onClientConnected(const QHostAddress& address)
{
    activeClients_++;
    connLabel_->setText(QString("外部算法节点：%1").arg(activeClients_));
    consoleText_->append(QString("<font color='#58a6ff'>[网络] Python/ROS 算法节点已接入: %1</font>").arg(address.toString()));
}

void MainWindow::onClientDisconnected(const QHostAddress& address)
{
    activeClients_ = qMax(0, activeClients_ - 1);
    connLabel_->setText(QString("外部算法节点：%1").arg(activeClients_));
    consoleText_->append(QString("<font color='#f85149'>[网络] 算法节点已断开: %1</font>").arg(address.toString()));
}

void MainWindow::onNetworkDataReceived(const QByteArray& data, const QHostAddress& address)
{
    QString msg = QString::fromUtf8(data).left(50);
    if (data.size() > 50) msg += "...";
    consoleText_->append(QString("<font color='#d2a8ff'>[RX %1] %2</font>").arg(address.toString()).arg(msg));
}

void MainWindow::onOpenScene() { statusBar()->showMessage("功能开发中..."); }
void MainWindow::onSaveScene() { statusBar()->showMessage("功能开发中..."); }
void MainWindow::onRecordData() { statusBar()->showMessage("功能开发中..."); }
void MainWindow::onExit() { close(); }

void MainWindow::closeEvent(QCloseEvent* event)
{
    stopBackend();
    event->accept();
}
