/**
 * @file Logger.h
 * @brief 日志系统头文件
 * 
 * 功能：异步日志记录
 * 
 * 八股知识点：
 * - 单例模式
 * - 文件 I/O
 * - 线程安全
 * - 可变参数
 */

#ifndef LOGGER_H
#define LOGGER_H

#include <QString>
#include <QFile>
#include <QMutex>
#include <QDateTime>

// 日志等级
enum LogLevel {
    LOG_DEBUG = 0,
    LOG_INFO = 1,
    LOG_WARN = 2,
    LOG_ERROR = 3
};

// 日志宏
#define LOG_DEBUG(...) Logger::log(LOG_DEBUG, __FILE__, __LINE__, __VA_ARGS__)
#define LOG_INFO(...)  Logger::log(LOG_INFO,  __FILE__, __LINE__, __VA_ARGS__)
#define LOG_WARN(...)  Logger::log(LOG_WARN,  __FILE__, __LINE__, __VA_ARGS__)
#define LOG_ERROR(...) Logger::log(LOG_ERROR, __FILE__, __LINE__, __VA_ARGS__)

/**
 * @brief 日志类（单例）
 */
class Logger
{
public:
    /**
     * @brief 初始化日志系统
     * @param filename 日志文件路径
     */
    static void init(const QString& filename);
    
    /**
     * @brief 关闭日志系统
     */
    static void shutdown();
    
    /**
     * @brief 记录日志
     * @param level 日志等级
     * @param file 源文件
     * @param line 行号
     * @param format 格式字符串
     * @param ... 参数
     */
    static void log(LogLevel level, const char* file, int line, 
                    const char* format, ...);
    
    /**
     * @brief 设置日志等级
     * @param level 最低日志等级
     */
    static void setLevel(LogLevel level);
    
private:
    Logger();
    ~Logger();
    
    /**
     * @brief 写入日志
     */
    void write(LogLevel level, const QString& message);
    
    /**
     * @brief 获取日志等级字符串
     */
    static const char* levelToString(LogLevel level);
    
private:
    static Logger* instance_;
    static QMutex mutex_;
    
    QFile file_;
    LogLevel minLevel_;
};

#endif // LOGGER_H
