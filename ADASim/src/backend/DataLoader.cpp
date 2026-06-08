/**
 * @file DataLoader.cpp
 * @brief 自动驾驶算法仿真平台 (ADASim) - 数据流引擎实现
 * @date 2026-04
 * @details
 * 本文件实现了基于“无限滚动沙盒 + 行车记录仪回溯 (Rolling Window Replay Buffer)”的数据驱动引擎。
 * * * 【说明】
 * 1. 死代码消除：彻底移除了早期版本遗留的 SQLite 读取逻辑，全面拥抱纯内存驱动的沙盒生成器。
 * 2. 跨线程安全性：严格规范了 QTimer 的生命周期，确保数据引擎在独立子线程中高频且稳定地分发数据。
 * 3. 完美回放体验：重构了 start() 与 stop() 逻辑，支持无缝暂停与继续播放；结合 FIFO 队列保障内存不溢出。
 */

#include "DataLoader.h"
#include "common/Logger.h" // 接入 ADASim 统一日志系统

#include <QThread>
#include <QDebug>

// ============================================================================
// [构造与析构]
// ============================================================================
DataLoader::DataLoader(const QString& dataPath, QObject *parent)
    : QObject(parent)
    , dataPath_(dataPath)
    , replayCursor_(-1)  // 初始化为 -1，代表当前处于生成最新帧的现实时间线
    , isRunning_(false)
    , timer_(new QTimer(this))
{
    lastTimestamp_=std::chrono::steady_clock::now();
    // 【多线程安全基石】：注册自定义的泛型容器，允许它们在子线程与主线程之间安全排队传递
    qRegisterMetaType<QVector<QPointF>>("QVector<QPointF>");

    // 绑定定时器的高频滴答事件到数据泵核心函数
    connect(timer_, &QTimer::timeout, this, &DataLoader::loadNextFrame);
    
    LOG_INFO("DataLoader 底层数据引擎已初始化 (数据加载路径: %s)", dataPath.toStdString().c_str());
}

DataLoader::~DataLoader()
{
    stop();
    LOG_INFO("DataLoader 数据引擎已安全释放");
}

// ============================================================================
// [引擎生命周期控制]
// ============================================================================
void DataLoader::start()
{
    // 【跨线程防错保护】：强制检查调用者所在的线程是否与 DataLoader 归属的子线程一致。
    if (QThread::currentThread() != this->thread()) {
        LOG_ERROR("致命架构错误：尝试跨线程直接启动 DataLoader 定时器！请使用信号槽或 invokeMethod。");
        return;
    }

    // 💡【核心修复】：这里不再清空 historyQueue_ 和坐标，使得 start 具备“继续 (Resume)”功能。
    // 如果用户是从暂停状态、或者拖拽进度条后点击“开始”，将顺滑地接着播放。

    if (!isRunning_) {
        isRunning_ = true;
        // 以 10Hz (100ms) 的高频触发数据分发流
        timer_->start(100);  
        
        emit statusUpdate("正在运行：仿真推演中...");
        
        if (replayCursor_ == -1) {
            LOG_INFO("数据引擎启动：顺接当前时间线，继续沙盒生成模式");
        } else {
            LOG_INFO("数据引擎恢复：从历史第 %d 帧开始无缝续播", replayCursor_);
        }
    }
}

void DataLoader::pause()
{
    if (isRunning_) {
        isRunning_ = false;
        timer_->stop();
        
        emit statusUpdate("引擎已暂停");
        LOG_INFO("数据引擎已触发安全暂停");
    }
}

void DataLoader::stop()
{
    isRunning_ = false;
    if (timer_->isActive()) {
        timer_->stop();
    }
    
    // 💡【重置逻辑归位】：真正需要“从头开始”时，用户必须点击停止。这里负责彻底清空内存状态。
    historyQueue_.clear();
    liveSandboxX_ = 0.0;
    replayCursor_ = -1;
    
    // 重置前端 UI 的进度条状态
    emit totalFramesLoaded(0);
    emit currentFrameUpdated(0);
    emit statusUpdate("引擎已停止并重置");
    
    LOG_INFO("数据引擎已完全停止，并清空所有内存缓冲帧");
}

// ============================================================================
// [核心数据泵与智能分流机制]
// ============================================================================
void DataLoader::loadNextFrame() 
{
    if (!isRunning_) return;

    // 【核心状态机：智能分流】
    // 判断当前游标 replayCursor_ 是否合法且小于队列末尾，这意味着用户正在“回顾历史”
    
    auto now = std::chrono::steady_clock::now();
    double dt = std::chrono::duration<double>(now-lastTimestamp_).count();
    lastTimestamp_=now;
    LOG_DEBUG("dt= %f seconds",dt);
    
    if (replayCursor_ >= 0 && replayCursor_ < historyQueue_.size() - 1) {
        
        // --------------------------------------------------------
        // 模式 A：历史回放 (播放 FIFO 队列中的缓存帧)
        // --------------------------------------------------------
        replayCursor_++; // 游标自然地推向下一个历史时刻
        FrameState state = historyQueue_[replayCursor_];
        
        // 将历史状态分发给规控中枢与渲染层
        emit vehiclePositionReady(state.x, state.y, state.yaw,0.1);
        emit pointCloudReady(QVector<QPointF>()); // 纯净沙盒背景，等待 Python 打入虚拟障碍物
        
        // 驱动前端进度条前进，但不去扩展滑动条的总上限
        emit currentFrameUpdated(replayCursor_);
        
    } else {
        
        // --------------------------------------------------------
        // 模式 B：探索未来 (正常沙盒物理推演，无尽生成)
        // --------------------------------------------------------
        replayCursor_ = -1; // 解除回放状态，标记游标已追上最新现实时间线

        // 1. 简易物理引擎推演：沙盒车辆以约 18km/h 的速度无限向前行驶
        const double TARGET_SPEED=5.0;
        double deltaX =TARGET_SPEED *dt;
        liveSandboxX_ +=deltaX;
        
        FrameState state = {liveSandboxX_, 0.0, 0.0};
        
        // 2. 将最新产生的一帧状态存入“行车记录仪”缓存尾部
        historyQueue_.append(state);

        // 3. 严格限制内存消耗：FIFO (先进先出) 滚动剔除老旧数据，维持最近 100 帧
        if (historyQueue_.size() > 100) {
            historyQueue_.pop_front();
        }

        // 4. 将最新状态分发出去
        emit vehiclePositionReady(state.x, state.y, state.yaw,dt);
        emit pointCloudReady(QVector<QPointF>());

        // 5. 动态扩展前端进度条的上限边界，并迫使滑块永远顶在最右侧
        emit totalFramesLoaded(historyQueue_.size());
        emit currentFrameUpdated(historyQueue_.size() - 1); 
    }
}

// ============================================================================
// [时光倒流交互响应]
// ============================================================================
void DataLoader::seekToFrame(int index) 
{
    // 拦截越界游标，确保目标帧在合法的内存队列范围内
    if (index >= 0 && index < historyQueue_.size()) {
        
        // 【时光倒流核心】：剥离当前真实时间线，将引擎渲染游标强制拉回过去
        replayCursor_ = index; 
        
        // 立刻从缓存队列中抽出那一瞬的历史帧，并渲染到画面上实现“所拖即所见”
        FrameState state = historyQueue_[index];
        emit vehiclePositionReady(state.x, state.y, state.yaw,0.1);
        emit pointCloudReady(QVector<QPointF>());
        
        LOG_INFO("上帝之手触发：系统画面被手动回溯至历史队列的第 %d 帧", index);
    }
}
