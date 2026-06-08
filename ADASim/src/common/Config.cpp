/**
 * @file Config.cpp
 * @brief 自动驾驶算法仿真平台 (ADASim) - 全局配置管理器实现
 * @date 2026-04
 * @details
 * 本文件实现了基于 JSON 的线程安全全局配置管理器。负责整个系统的参数加载、动态读取与运行时修改。
 * * 【架构说明】
 * 1. 单例模式安全化：采用 Meyers Singleton（局部静态变量）范式，确保 C++11 标准下的线程安全初始化，且杜绝内存泄漏。
 * 2. 引入读写分离锁 (QReadWriteLock)：彻底消除了原架构中 TCP 子线程更新配置与 GUI 主线程读取配置时的并发冲突（Segfault）。
 * 3. 性能最优化：读操作加读锁（允许多个模块无阻塞同时读取），写操作加写锁（独占排他），在保障绝对安全的同时将性能损耗降至最低。
 */

#include "Config.h"
#include "common/Logger.h" // 接入 ADASim 统一日志系统

#include <QFile>
#include <cassert>
#include <QJsonDocument>
#include <QJsonObject>
#include <QReadLocker>
#include <QWriteLocker>

// ============================================================================
// [单例实例化]
// ============================================================================
Config& Config::instance()
{
    // 采用 Meyers 单例模式：利用 C++11 标准保证静态局部变量初始化的线程安全性
    static Config instance;
    return instance;
}

// ============================================================================
// [核心功能：加载配置文件]
// ============================================================================
static QVariantMap getDefaultConfig()
{
    QVariantMap defaults;
    defaults["wheelbase"] = 2.8;
    defaults["max_speed"] = 15.0;
    defaults["max_steering"] = 0.5;
    defaults["tcp_port"] = 8080;
    defaults["udp_port"] = 8081;
    defaults["lidar_range"] = 50.0;
    defaults["cluster_tolerance"] = 0.5;
    return defaults;
}

bool Config::load(const QString& filename)
{
    QFile file(filename);
    if (!file.open(QIODevice::ReadOnly)) {
        LOG_ERROR("配置文件读取失败，路径不存在或无权限: %s", filename.toStdString().c_str());
        return false;
    }
    
    QByteArray data = file.readAll();
    file.close();
    
    // 解析 JSON 文档
    QJsonParseError parseError;
    QJsonDocument doc = QJsonDocument::fromJson(data, &parseError);
    
    if (doc.isNull() || !doc.isObject()) {
        LOG_ERROR("配置文件 JSON 格式非法 (偏移量: %d): %s", parseError.offset, parseError.errorString().toStdString().c_str());
        return false;
    }
    
    // 如果需要对旧版本的配置进行兼容，或者合并默认配置，
    QVariantMap defaultConfig = getDefaultConfig();
    QVariantMap userConfig = doc.object().toVariantMap();
    for (auto it = userConfig.begin(); it != userConfig.end(); ++it) {
        defaultConfig[it.key()] = it.value();   // 用户配置覆盖默认
    }
    
    // 【线程安全层】：加写锁，独占锁定。此时阻止任何子模块（如传感器、算法）读取旧配置
    QWriteLocker locker(&lock_);
    config_ = defaultConfig;
    
    LOG_INFO("成功加载系统全局配置: %s", filename.toStdString().c_str());
    return true;
}

// ============================================================================
// [核心功能：高频安全读取]
// ============================================================================
QVariant Config::get(const QString& key, const QVariant& defaultValue) const
{
    // 【线程安全层】：加读锁，共享锁定。
    // 允许多个模块（如 View2D 渲染、ObstacleDetector 推理）在同一微秒内高频读取配置，互不阻塞；
    // 但如果有其他线程正在调用 set() 写入数据，则会短暂阻塞直到写入完成。
    QReadLocker locker(&lock_);
    return config_.value(key, defaultValue);
}

// ============================================================================
// [核心功能：运行时安全修改]
// ============================================================================
void Config::set(const QString& key, const QVariant& value)
{
    // 【线程安全层】：加写锁，独占锁定。
    // 确保如 TCP Socket 接收到远端调参指令时，能安全地写入配置字典而不破坏内存结构。
    QWriteLocker locker(&lock_);
    config_[key] = value;
}
