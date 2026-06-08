/**
 * @file Logger.cpp
 * @brief 日志系统实现文件
 */

#include "Logger.h"
#include <QTextStream>
#include <QDir>
#include <cstdarg>
#include <cstdio>

Logger* Logger::instance_ = nullptr;
QMutex Logger::mutex_;

Logger::Logger()
    : minLevel_(LOG_DEBUG)
{
}

Logger::~Logger()
{
    if (file_.isOpen()) {
        file_.close();
    }
}

void Logger::init(const QString& filename)
{
    QMutexLocker locker(&mutex_);
    
    if (instance_) {
        return;
    }
    
    instance_ = new Logger();
    
    // 创建日志目录
    QFileInfo fileInfo(filename);
    QDir dir = fileInfo.absoluteDir();
    if (!dir.exists()) {
        dir.mkpath(".");
    }
    
    // 打开文件
    instance_->file_.setFileName(filename);
    if (instance_->file_.open(QIODevice::WriteOnly | QIODevice::Text)) {
        instance_->file_.resize(0);  // 清空文件
    }
}

void Logger::shutdown()
{
    QMutexLocker locker(&mutex_);
    
    if (instance_) {
        if (instance_->file_.isOpen()) {
            instance_->file_.close();
        }
        delete instance_;
        instance_ = nullptr;
    }
}

void Logger::log(LogLevel level, const char* file, int line, 
                 const char* format, ...)
{
    QMutexLocker locker(&mutex_);
    
    if (!instance_ || level < instance_->minLevel_) {
        return;
    }
    
    // 格式化消息
    char message[1024];
    va_list args;
    va_start(args, format);
    vsnprintf(message, sizeof(message), format, args);
    va_end(args);
    
    // 提取文件名
    QString fileName = QFileInfo(file).fileName();
    
    // 构建完整日志行
    QString logLine = QString("[%1] [%2] [%3:%4] %5")
        .arg(QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss.zzz"))
        .arg(levelToString(level))
        .arg(fileName)
        .arg(line)
        .arg(message);
    
    instance_->write(level, logLine);
    
    // 同时输出到控制台
    printf("%s\n", logLine.toStdString().c_str());
    fflush(stdout);
}

void Logger::setLevel(LogLevel level)
{
    QMutexLocker locker(&mutex_);
    
    if (instance_) {
        instance_->minLevel_ = level;
    }
}

void Logger::write(LogLevel level, const QString& message)
{
    if (file_.isOpen()) {
        QTextStream out(&file_);
        out << message << "\n";
        out.flush();
    }
}

const char* Logger::levelToString(LogLevel level)
{
    switch (level) {
        case LOG_DEBUG: return "DEBUG";
        case LOG_INFO:  return "INFO";
        case LOG_WARN:  return "WARN";
        case LOG_ERROR: return "ERROR";
        default:        return "UNKNOWN";
    }
}
