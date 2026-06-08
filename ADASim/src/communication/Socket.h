/**
 * @file Socket.h
 * @brief 自动驾驶算法仿真平台 (ADASim) - 网络通信中枢头文件
 * @date 2026-04
 * @details
 * 本模块定义了仿真平台的底层网络通信引擎，负责与外部算法大脑（如 Python 规控脚本）建立全双工通信。
 * * 【架构说明】
 * 1. 内存防漏机制：引入 onClientDisconnected 专用槽函数，统一接管远端断开、网络异常等事件，利用 Qt 的 deleteLater() 彻底杜绝 QTcpSocket 的内存泄漏（悬挂指针）。
 * 2. 异步非阻塞：严格依赖 Qt 的底层事件循环驱动，将网络 I/O 与高耗时的业务解耦。
 * 3. 混合协议支持：同时支持 TCP (适合规控命令下发等高可靠性需求) 和 UDP (适合高频点云广播等对时延敏感的数据流)。
 */

#ifndef SOCKET_H
#define SOCKET_H

#include <QObject>
#include <QTcpServer>
#include <QTcpSocket>
#include <QUdpSocket>
#include <QHostAddress>
#include <QByteArray>
#include <QList>

/**
 * @class SocketServer
 * @brief 自动驾驶通信网关服务器
 * * 作为一个桥梁，将 C++ 底层的物理引擎、感知数据，与上层的高级算法（Python 或 C++ ROS）无缝连接。
 */
class SocketServer : public QObject
{
    Q_OBJECT
    
public:
    /**
     * @brief 构造函数
     * @param parent 父对象指针（常用于管理 QThread 的生命周期树）
     */
    explicit SocketServer(QObject *parent = nullptr);
    
    /**
     * @brief 析构函数，断开所有存活连接并安全释放端口资源
     */
    ~SocketServer();
    
public slots:
    // ==========================================
    // [服务控制 API] 允许跨线程安全调用 (QueuedConnection)
    // ==========================================
    
    /**
     * @brief 启动 TCP 服务端 (用于可靠通信，如命令控制、规控轨迹下发)
     * @param port 监听的本地端口号
     * @return bool 是否成功绑定并开启监听
     */
    bool startTcpServer(quint16 port);
    
    /**
     * @brief 启动 UDP 服务端 (用于高速广播，如雷达点云/视频流推送)
     * @param port 监听的本地端口号
     * @return bool 是否成功绑定
     */
    bool startUdpServer(quint16 port);
    
    /**
     * @brief 停止所有网络服务，踢出全部客户端并回收套接字资源
     */
    void stop();
    
    /**
     * @brief 向所有活跃的 TCP 客户端广播数据
     * @param data 待发送的二进制或 JSON 字节序列
     * * 💡【拓展功能接口】：未来如果接入多算法模型对比（例如同时连接两个 Python 客户端跑不同算法进行对比），
     * 可以扩展重载函数：sendToClient(const QByteArray& data, const QHostAddress& targetIp) 实现点对点定向下发。
     */
    void sendToClient(const QByteArray& data);
    
signals:
    // ==========================================
    // [网络事件信号] 向上层模块 (如 MainWindow) 抛出网络状态
    // ==========================================
    
    /**
     * @brief 接收到客户端数据时触发
     * @param data 收到的数据报文
     * * 💡【拓展功能接口】：TCP 是流式传输，底层可能会出现“粘包/半包”现象。
     * 建议在更高层的业务逻辑（如 DataParser 模块）中基于 JSON 换行符 `\n` 或自定义包头进行粘包切分。
     * @param address 发送方的 IP 地址
     */
    void dataReceived(const QByteArray& data, const QHostAddress& address);
    
    /** @brief 新客户端成功建立 TCP 连接时触发 */
    void clientConnected(const QHostAddress& address);
    
    /** @brief 客户端断开连接（主动退出、心跳超时或网络异常崩溃）时触发 */
    void clientDisconnected(const QHostAddress& address);
    
private slots:
    // ==========================================
    // [底层系统事件响应] 拦截 Qt 底层的网络触发器
    // ==========================================
    
    void onNewConnection();       ///< 响应 QTcpServer::newConnection，分配新 Socket
    void onReadyRead();           ///< 响应 QTcpSocket::readyRead，读取 TCP 流
    void onUdpDataReceived();     ///< 响应 QUdpSocket::readyRead，读取 UDP 数据报
    
    /**
     * @brief 【内存安全核心】统一处理客户端的断开事件
     * * 从 clients_ 列表中移除对应 socket，并调用 deleteLater() 交由事件循环安全回收内存。
     */
    void onClientDisconnected(); 
    void sendPing(QTcpSocket* client); 
    
private:
    // ==========================================
    // [网络资源与状态缓存区]
    // * 遵循 Modern C++ 规范，对指针进行缺省 nullptr 初始化，防止野指针
    // ==========================================
    
    QTcpServer* tcpServer_ = nullptr;    ///< TCP 监听守护句柄
    QUdpSocket* udpSocket_ = nullptr;    ///< UDP 数据收发句柄
    struct ClientInfo {
        QTcpSocket* socket;
        QTimer* heartbeatTimer;
        int missedPongs;
    };

    QList<ClientInfo> clients_;   // 替换原来的 QList<QTcpSocket*> clients_
    const int PING_INTERVAL_MS = 5000;   // 5秒发送一次 PING
    const int MAX_MISSED_PONGS = 3;      // 连续3次无响应则断开
};

#endif // SOCKET_H
