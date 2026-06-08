/**
 * @file Config.h
 * @brief 自动驾驶算法仿真平台 (ADASim) - 全局配置管理器头文件
 * @date 2026-04
 * @details
 * 本文件定义了基于 JSON 的线程安全全局配置管理器类。
 * * * 【说明】
 * 1. 并发安全：引入 QReadWriteLock 保证在多线程环境（如 GUI 线程高频读取，Socket 线程异步写入）下的绝对内存安全。
 * 2. 实例约束：通过 `= delete` 禁用拷贝构造和赋值操作符，严格防范单例在传递中被破坏。
 * 3. 语义严谨：使用 `mutable` 关键字修饰读写锁，使得在对外承诺不修改数据状态的 `const` 成员函数（如 `get`）内部依然可以加读锁，完美契合 C++ 标准语义。
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <QString>
#include <QVariantMap>
#include <QReadWriteLock>

/**
 * @class Config
 * @brief 线程安全的单例配置管理类
 * * 负责解析 JSON 配置文件，并在系统全生命周期内为感知、规控、UI 渲染等模块提供高并发安全的读写支持。
 */
class Config {
public:
    /**
     * @brief 获取系统全局的配置单例
     * @return Config& 单例对象的引用
     * @note 内部采用 C++11 局部静态变量 (Meyers Singleton) 实现，天然具备初始化时的线程安全性。
     */
    static Config& instance();

    // 禁用拷贝构造和赋值操作符，防止单例被意外复制，造成状态割裂
    Config(const Config&) = delete;
    Config& operator=(const Config&) = delete;

    /**
     * @brief 从 JSON 文件加载配置到内存中
     * @param filename 配置文件绝对路径或相对路径
     * @return bool 加载及解析是否成功
     * * 💡【拓展功能接口】：如果后期在实车部署时需要支持 YAML 格式，或对接 ROS 参数服务器 (ROS Param Server)，
     * 可在此处新增如 loadFromYaml() 或 fetchFromRosServer() 的重载接口。
     */
    bool load(const QString& filename);

    /**
     * @brief 线程安全地读取指定的配置项
     * @param key 配置的键名
     * @param defaultValue 当键名不存在或发生读取错误时返回的后备默认值
     * @return QVariant 配置项的泛型值，外部获取后可灵活转为 toInt(), toDouble(), toString() 等
     * * 💡【拓展功能接口】：当前 key 是一级字符串。为了适配复杂的自动驾驶参数结构，
     * 后续可以升级此函数支持类似于 "sensor/lidar/fov" 的嵌套键解析。
     */
    QVariant get(const QString& key, const QVariant& defaultValue = QVariant()) const;

    /**
     * @brief 线程安全地在运行时动态修改或添加配置项
     * @param key 配置的键名
     * @param value 欲修改或写入的新值
     * * 💡【拓展功能接口】：为支持“所见即所得”的调参体验，未来可让 Config 继承 QObject，
     * 并在 set() 被调用后发射一个 `emit configChanged(key, value)` 信号，让全网模块自动热更新。
     */
    void set(const QString& key, const QVariant& value);

private:
    /**
     * @brief 私有化默认构造函数与析构函数，强制所有模块必须通过 instance() 访问
     */
    Config() = default;
    ~Config() = default;

    QVariantMap config_;                 ///< 存储全局配置数据的内存字典
    mutable QReadWriteLock lock_;        ///< 读写锁 (mutable 修饰使其在 const 函数 get() 中亦可操作)
};

#endif // CONFIG_H