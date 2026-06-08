/**
 * @file Socket.cpp
 * @brief 自动驾驶算法仿真平台 (ADASim) - 网络通信中枢实现
 * @date 2026-04
 * @details
 * 本模块实现了底层 TCP/UDP 服务的生命周期管理与报文收发。
 * * 【亮点】
 * 1. 悬挂指针免疫：通过 onClientDisconnected 拦截底层的断开事件，安全移出连接池，彻底消灭了给已断开的 Socket 发送数据导致的 Crash。
 * 2. 严格的线程隔离检查：强制 QThread::currentThread() 校验，防止主线程直接操作属于 Backend 线程的 Socket 对象（违反 Qt 对象亲和性原则）。
 * 3. 优雅降级与资源回收：stop() 函数中对所有未断开的 Client 发送 abort() 并强制释放，确保应用程序退出时没有任何隐式内存泄漏或僵尸端口占用。
 */

#include "Socket.h"
#include "common/Logger.h" // 接入 ADASim 全局日志系统

#include <QThread>

static const int PING_INTERVAL_MS = 5000;
static const int MAX_MISSED_PONGS = 3;

// ============================================================================
// [构造与析构]
// ============================================================================

SocketServer::SocketServer(QObject *parent)
    : QObject(parent)
    , tcpServer_(nullptr)
    , udpSocket_(nullptr)
{
    // 初始化时仅分配空指针，实际的句柄绑定推迟到 start() 中执行
}

SocketServer::~SocketServer()
{
    stop();
    LOG_INFO("SocketServer: 通信网关底座已安全销毁");
}

// ============================================================================
// [服务生命周期管理]
// ============================================================================

bool SocketServer::startTcpServer(quint16 port)
{
    // 【架构防线】：确保网络操作在所属的子线程事件循环中执行
    if (QThread::currentThread() != this->thread()) {
        LOG_ERROR("架构违规：尝试跨线程启动 TCP Server！请使用信号/invokeMethod(QueuedConnection)。");
        return false; 
    }

    if (tcpServer_) {
        LOG_WARN("TCP Server 已在运行，忽略重复启动请求");
        return true;
    }

    tcpServer_ = new QTcpServer(this);
    
    connect(tcpServer_, &QTcpServer::newConnection,
            this, &SocketServer::onNewConnection);
    
    if (!tcpServer_->listen(QHostAddress::Any, port)) {
        LOG_ERROR("TCP 服务器绑定端口 %d 失败: %s", port, tcpServer_->errorString().toStdString().c_str());
        delete tcpServer_;
        tcpServer_ = nullptr;
        return false;
    }
    
    LOG_INFO("✅ TCP 通信网关已就绪，正在监听指令端口: %d", port);
    return true;
}

bool SocketServer::startUdpServer(quint16 port)
{
    if (QThread::currentThread() != this->thread()) {
        LOG_ERROR("架构违规：尝试跨线程启动 UDP Server！");
        return false;
    }

    if (udpSocket_) return true;

    udpSocket_ = new QUdpSocket(this);
    
    connect(udpSocket_, &QUdpSocket::readyRead,
            this, &SocketServer::onUdpDataReceived);
    
    // ShareAddress：允许多个服务绑定同一端口，这对于同机运行的多个仿真节点(如 ROS 节点)接收广播至关重要
    if (!udpSocket_->bind(QHostAddress::Any, port, QUdpSocket::ShareAddress)) {
        LOG_ERROR("UDP 服务器绑定端口 %d 失败: %s", port, udpSocket_->errorString().toStdString().c_str());
        delete udpSocket_;
        udpSocket_ = nullptr;
        return false;
    }
    
    LOG_INFO("✅ UDP 数据广播网关已就绪，正在监听高速端口: %d", port);
    return true;
}

void SocketServer::stop()
{
    // 1. 关闭 TCP 服务端守护进程，拒绝新接入的握手
    if (tcpServer_) {
        tcpServer_->close();
        tcpServer_->deleteLater();
        tcpServer_ = nullptr;
    }
    
    // 2. 【核心内存清理】：强制切断并释放所有处于活跃状态的 Client
    for (auto& info : clients_) {
        if (info.socket) {
            info.socket->disconnect();
            info.socket->abort();
            info.socket->deleteLater();
        }
        if (info.heartbeatTimer) {
            info.heartbeatTimer->stop();
            info.heartbeatTimer->deleteLater();
        }
    }
    clients_.clear();
    
    // 3. 关闭 UDP 接收套接字
    if (udpSocket_) {
        udpSocket_->close();
        udpSocket_->deleteLater();
        udpSocket_ = nullptr;
    }
    
    LOG_INFO("SocketServer: 所有网络通信服务已安全熔断，端口资源已回收");
}

// ============================================================================
// [数据发送 API]
// ============================================================================

void SocketServer::sendToClient(const QByteArray& data)
{
    for (const auto& info : clients_) {
       if (info.socket && info.socket->state() == QAbstractSocket::ConnectedState) {
            info.socket->write(data);
            info.socket->flush();
        }
    }
}

// ============================================================================
// [底层事件路由机制]
// ============================================================================

void SocketServer::onNewConnection()
{
    while (tcpServer_->hasPendingConnections()) {
        QTcpSocket* client = tcpServer_->nextPendingConnection();
        
        // 绑定读数据信号
        connect(client, &QTcpSocket::readyRead,
                this, &SocketServer::onReadyRead);
                
        // 【架构基石】：将断开事件路由到自定义的清理中心，而不是让它自生自灭
        connect(client, &QTcpSocket::disconnected,
                this, &SocketServer::onClientDisconnected);
                
        // 创建心跳定时器
        QTimer* timer = new QTimer(this);
        timer->setInterval(PING_INTERVAL_MS);
        
        //使用 lambda 捕获 client
        connect(timer, &QTimer::timeout, this, [this, client]() {
            sendPing(client);
        });
        timer->start();

        // 存储客户端信息
        ClientInfo info{client, timer, 0};
        clients_.append(info);
        
        LOG_INFO("🔗 新算法客户端接入: %s | 当前并发连接数: %d", 
                 client->peerAddress().toString().toStdString().c_str(), clients_.size());
                 
        emit clientConnected(client->peerAddress());
    }
}

void SocketServer::sendPing(QTcpSocket* client)
{
    LOG_DEBUG("Sending PING to %s", client->peerAddress().toString().toStdString().c_str());
    if (!client || client->state() != QAbstractSocket::ConnectedState)
        return;

    // 发送 PING 报文
    client->write("{\"type\":\"PING\"}\n");
    client->flush();

    // 查找对应的 ClientInfo，增加 missed 计数
    for (auto& info : clients_) {
        if (info.socket == client) {
            info.missedPongs++;
            // 如果连续超时次数超过阈值，主动断开
            if (info.missedPongs >= MAX_MISSED_PONGS) {
                LOG_WARN("客户端心跳超时，主动断开: %s", client->peerAddress().toString().toStdString().c_str());
                client->disconnectFromHost();  // 触发 disconnected 信号，进而清理
            }
            break;
        }
    }
}

void SocketServer::onClientDisconnected()
{
    QTcpSocket* client = qobject_cast<QTcpSocket*>(sender());
    if (!client) return;
    // 查找并移除对应的 ClientInfo
    for (int i = 0; i < clients_.size(); ++i) {
        if (clients_[i].socket == client) {
            // 停止并删除定时器
            clients_[i].heartbeatTimer->stop();
            clients_[i].heartbeatTimer->deleteLater();
            clients_.removeAt(i);
            break;
        }
    }
    emit clientDisconnected(client->peerAddress());
    client->deleteLater();
}
    
void SocketServer::onReadyRead()
{
    QTcpSocket* client = qobject_cast<QTcpSocket*>(sender());
    if (!client) return;

    while (client->canReadLine()) {
        QByteArray line = client->readLine();
        
        // 简单判断是否为 PONG（可以解析 JSON，也可以直接检查字符串）
        if (line.trimmed() == "{\"type\":\"PONG\"}") {
        LOG_DEBUG("Received PONG from %s", client->peerAddress().toString().toStdString().c_str());
            // 重置 miss 计数
            for (auto& info : clients_) {
                if (info.socket == client) {
                    info.missedPongs = 0;
                    break;
                }
            }
        } else {
            // 其他数据正常发送 dataReceived 信号
            emit dataReceived(line, client->peerAddress());
        }
    }
}

void SocketServer::onUdpDataReceived()
{
    // 循环榨干底层的 UDP 接收缓冲区
    while (udpSocket_->hasPendingDatagrams()) {
        QByteArray datagram;
        datagram.resize(udpSocket_->pendingDatagramSize());
        
        QHostAddress sender;
        quint16 senderPort;
        
        // 零拷贝思想：直接读入 QByteArray 的底层指针
        udpSocket_->readDatagram(datagram.data(), datagram.size(),
                                 &sender, &senderPort);
        
        emit dataReceived(datagram, sender);
    }
