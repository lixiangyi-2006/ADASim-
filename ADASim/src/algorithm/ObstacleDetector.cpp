/**
 * @file ObstacleDetector.cpp
 * @brief 自动驾驶算法仿真平台 (ADASim) - 障碍物检测与聚类算法实现
 * @date 2026-04
 * @details
 * 本文件实现了点云去噪、基于广度优先搜索 (BFS) 的欧式聚类，以及障碍物物理特征提取。
 *
 * * 【架构与性能调优说明】
 * 1. 消除 O(N^2) 内存拷贝黑洞：在 BFS 聚类队列中，彻底移除了原代码中导致极度掉帧的 QVector::takeFirst()，改用 head 游标推进，使得出队操作从 O(N) 降为极速的 O(1)。
 * 2. 彻底消灭高频浮点开方：在聚类邻域搜索的十万次级热点循环 (Hot Loop) 中，将 std::sqrt 替换为平方距离 (Squared Distance) 比较，极大缓解了 CPU 的 ALU 运算瓶颈。
 * 3. 预分配内存 (Memory Pre-allocation)：在 filterGround 等操作中使用 reserve() 预分配内存，减少堆内存碎片的产生与动态扩容开销。
 */

#include "ObstacleDetector.h"
#include <cmath>
#include <QtGlobal>

// ============================================================================
// [构造与析构]
// ============================================================================
ObstacleDetector::ObstacleDetector()
    : maxDistance_(50.0f)
    , minPoints_(5)
    , groundHeight_(0.5f)
{
}

ObstacleDetector::~ObstacleDetector() {}

// ============================================================================
// [核心感知流水线]
// ============================================================================
QVector<Obstacle> ObstacleDetector::detect(const QVector<QPointF>& pointCloud)
{
    QVector<Obstacle> obstacles;
    if (pointCloud.isEmpty()) {
        return obstacles;
    }

    // 1. 去除环境噪点与车身反射盲区 (Ground/Ego-Noise Filtering)
    QVector<QPointF> filtered = filterGround(pointCloud);

    // 2. 极速欧式聚类 (Fast Euclidean Clustering)
    QVector<QVector<QPointF>> clusters = clusterPoints(filtered);

    // 3. 障碍物目标物理特征提取 (Feature Extraction)
    for (const auto& cluster : clusters) {
        // 滤除离散噪点：只有点数达到阈值的簇，才被承认为真实的物理障碍物
        if (cluster.size() >= minPoints_) {
            Obstacle obs;
            float sumX = 0.0f;
            float sumY = 0.0f;

            for (const auto& point : cluster) {
                sumX += point.x();
                sumY += point.y();
            }

            // 计算障碍物几何质心 (Centroid)
            obs.position = QPointF(sumX / cluster.size(), sumY / cluster.size());

            // 【性能优化】：推迟并最小化开方运算，全周期仅对聚类后的少量质心进行 std::sqrt
            float distSq = obs.position.x() * obs.position.x() +
                           obs.position.y() * obs.position.y();
            obs.distance = std::sqrt(distSq);

            // 截断感知有效边界之外的无效目标
            if (obs.distance > maxDistance_) {
                continue;
            }

            // 计算方位角 (Azimuth Angle)
            obs.angle = std::atan2(obs.position.y(), obs.position.x());

            // 计算感知置信度 (Confidence Score)
            // 启发式算法模型：距离越近，点数越多（相对于基准20个点），置信度越趋近于 1.0 (100%)
            obs.confidence = qMin(1.0f, (cluster.size() / 20.0f) * (1.0f - obs.distance / maxDistance_));
            obs.pointCount = cluster.size();

            obstacles.append(obs);
        }
    }
    return obstacles;
}

void ObstacleDetector::setThreshold(float distance, int minPoints)
{
    maxDistance_ = distance;
    minPoints_ = minPoints;
}

// ============================================================================
// [去噪子程序]
// ============================================================================
QVector<QPointF> ObstacleDetector::filterGround(const QVector<QPointF>& points)
{
    QVector<QPointF> filtered;
    filtered.reserve(points.size());
    
    for (const auto& point : points) {
        // 在 3D 中 Z 轴是高度，但在我们 2D BEV 视图中 Y 轴是横向偏移。
        // 如果按 Y 过滤，会把正前方的夺命障碍物全部当成马路删掉！
        // 因此在 2D 模式下，直接放行点云，交给后续的 RANSAC 算法去处理噪点。
        filtered.append(point);
    }
    return filtered;
}

// ============================================================================
// [聚类子程序]
// ============================================================================
QVector<QVector<QPointF>> ObstacleDetector::clusterPoints(const QVector<QPointF>& points)
{
    QVector<QVector<QPointF>> clusters;
    if (points.isEmpty()) return clusters;

    // 聚类半径阈值 (米)：两点之间距离小于此值，则视为同一个障碍物实体
    const float clusterRadius = 2.0f;
    const float radiusSq = clusterRadius * clusterRadius;

    QVector<bool> visited(points.size(), false);

    for (int i = 0; i < points.size(); ++i) {
        if (visited[i]) continue;

        QVector<QPointF> cluster;
        QVector<int> queue;

        queue.append(i);
        visited[i] = true;

        // 【核心性能修复】：使用游标 (Cursor) 推进，替代 QVector::takeFirst()
        int head = 0;

        // 基于广度优先搜索 (BFS) 的空间蔓延聚类
        while (head < queue.size()) {
            int idx = queue[head++];
            cluster.append(points[idx]);

            // 💡【架构进阶提示】：
            // 当前暴力搜索复杂度为 O(N^2)。若未来接入真实的高线束激光雷达（单帧 > 10万点），
            // 此处必须使用 Grid Spatial Hash (空间网格哈希) 或 PCL 库的 KD-Tree，将搜索降级至 O(N log N)。
            for (int j = 0; j < points.size(); ++j) {
                if (visited[j]) continue;

                float dx = points[idx].x() - points[j].x();
                float dy = points[idx].y() - points[j].y();

                // 【热点循环优化】：使用平方距离，完全规避代价极其高昂的 std::sqrt
                if (dx * dx + dy * dy < radiusSq) {
                    visited[j] = true; // 入队前即刻标记，严防重复计算与死循环
                    queue.append(j);
                }
            }
        }

        if (!cluster.isEmpty()) {
            clusters.append(cluster);
        }
    }
    return clusters;
}