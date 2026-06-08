/**
 * @file main.cpp
 * @brief 自动驾驶算法仿真平台 (Autonomous Driving Algorithm Simulation Platform, 简称: ADASim) - 程序入口
 * @date 2026-04
 * * @details
 * 本文件是整个系统的主入口，负责应用程序的初始化、参数解析以及主事件循环的启动。
 */

#include <QApplication>
#include <QCommandLineParser>
#include <QCommandLineOption>
#include <QDir>

#include "gui/MainWindow.h"
#include "common/Config.h"
#include "common/Logger.h"

int main(int argc, char *argv[])
{
    // 1. 创建应用程序实例（⚠️注意：必须在所有 GUI 组件及资源加载之前初始化）
    QApplication app(argc, argv);
    
    // 2. 配置应用程序元数据（便于操作系统注册、日志标识及 QSettings 读取默认配置）
    QApplication::setApplicationName("ADASim");
    QApplication::setApplicationVersion("1.0.0");
    QApplication::setOrganizationName("ADASim Team");
    
    // 3. 初始化全局日志系统
    Logger::init("./logs/adasim.log");
    LOG_INFO("=== 自动驾驶算法仿真平台 (ADASim) 启动 ===");
    LOG_INFO("当前系统版本：%s", "1.0.0");
    
    // 4. 解析命令行参数（提供终端启动时的交互接口）
    // 💡【拓展功能接口】：未来如果需要增加“无界面渲染模式(Headless)”、“后台数据录制模式(Record)”等，
    // 可以直接在这里 addOption，并通过解析参数来决定是否跳过 MainWindow 的创建。
    QCommandLineParser parser;
    parser.setApplicationDescription("自动驾驶算法仿真平台 - 工业级架构教学与验证平台");
    parser.addHelpOption();
    parser.addVersionOption();
    
    // 设定配置文件路径选项 (-c, --config)
    QCommandLineOption configOption(QStringList() << "c" << "config",
                                    "指定全局配置文件路径", "config_path", "./data/config.json");
    parser.addOption(configOption);
    
    // 设定数据包路径选项 (-d, --data)
    QCommandLineOption dataOption(QStringList() << "d" << "data",
                                  "指定 SQLite 数据源与静态资源的加载目录", "data_path", "./data");
    parser.addOption(dataOption);
    
    parser.process(app);
    
    // 5. 路径规范化处理（将相对路径转换为绝对路径）
    // 目的：避免开发者在不同层级目录下运行可执行文件时，出现“找不到 dataset.db”的幽灵 Bug。
    QString configPath = parser.value(configOption);
    QString dataPath = parser.value(dataOption);
    
    if (!QDir::isAbsolutePath(configPath)) {
        configPath = QDir::cleanPath(QCoreApplication::applicationDirPath() + "/../" + configPath);
    }
    if (!QDir::isAbsolutePath(dataPath)) {
        dataPath = QDir::cleanPath(QCoreApplication::applicationDirPath() + "/../" + dataPath);
    }
    
    LOG_INFO("核心配置加载路径：%s", configPath.toStdString().c_str());
    LOG_INFO("底层数据加载路径：%s", dataPath.toStdString().c_str());
    
    // 6. 初始化全局配置管理器（采用线程安全单例模式，为全生命周期提供配置读取）
    if (!Config::instance().load(configPath)) {
        LOG_ERROR("加载配置文件失败，系统将以降级/默认配置继续运行！");
    }
    
    // 7. 实例化并显示主窗口
    // 💡【架构设计说明】：所有的核心业务（如 SQLite数据引擎、TCP/UDP通信层、Python算法联动层、2D物理渲染层）
    // 的初始化与子线程分配，均已深度封装在 MainWindow 内部。这保证了入口文件的极度整洁。
    MainWindow mainWindow(configPath, dataPath);
    mainWindow.show();
    LOG_INFO("主窗口界面渲染完成，准备进入 Qt 异步事件循环...");
    
    // 8. 移交控制权，进入主事件循环 (Event Loop)
    // 此时程序开始异步高效地响应鼠标交互、网络 TCP 报文与底层硬件定时器，直至窗口关闭。
    int ret = app.exec();
    
    // 9. 优雅退出与系统资源释放
    LOG_INFO("系统事件循环终止，应用程序安全退出，返回码：%d", ret);
    LOG_INFO("=== 自动驾驶算法仿真平台 (ADASim) 结束运行 ===");
    Logger::shutdown();
    
    return ret;
}