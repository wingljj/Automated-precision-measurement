#ifndef POSE_SORT_H
#define POSE_SORT_H

#include <array>
#include <cstddef>
#include <vector>

namespace trajsort {

/**
 * 使用 XYZ 位置（毫米）和 RX/RY/RZ 方向（弧度）表示的简单 6D 姿态，
 * 通常为约定的欧拉角。使用者可以将其映射到他们自己的姿态类型或从其映射。
 */
struct Pose {
    double x;
    double y;
    double z;
    double rx;
    double ry;
    double rz;
};

/**
 * 带有原始索引的姿态，用于跟踪排序操作。
 */
struct PoseWithIndex {
    Pose pose;
    std::size_t index;
};

/**
 * 使用加权 6D 空间中的最近邻算法对姿态列表进行排序以形成短路径。距离度量为：
 * sqrt( positionWeight^2 * ||d_pos||^2 + orientationWeight^2 * ||wrap(d_rot)||^2 )
 *
 * - 如果 startIndex 超出范围且 poses 非空，则默认为 0。
 * - 如果 poses.size() <= 1，则按原样返回副本。
 * - 旋转差值在每个轴上被包装到 [-pi, pi]。
 *
 * @param poses 输入的姿态列表
 * @param startIndex 可选的起始索引（默认为 0）
 * @param positionWeight XYZ 的权重（默认为 1.0）
 * @param orientationWeight RX/RY/RZ 的权重（默认为 0.0 -> 忽略旋转）
 * @return 重新排序的列表
 */
std::vector<Pose> sortPoses(const std::vector<Pose>& poses,
                            std::size_t startIndex = 0,
                            double positionWeight = 1.0,
                            double orientationWeight = 0.0);

/**
 * 根据相同的加权度量计算有序姿态的总路径长度。
 * 如果姿态数量少于 2 个，则返回 0。
 */
double totalPosePathLength(const std::vector<Pose>& orderedPoses,
                           double positionWeight = 1.0,
                           double orientationWeight = 0.0);

/**
 * 使用加权 6D 空间中的最近邻算法对带索引的姿态列表进行排序以形成短路径。距离度量为：
 * sqrt( positionWeight^2 * ||d_pos||^2 + orientationWeight^2 * ||wrap(d_rot)||^2 )
 *
 * - 如果 startIndex 超出范围且 poses 非空，则默认为 0。
 * - 如果 poses.size() <= 1，则按原样返回副本。
 * - 旋转差值在每个轴上被包装到 [-pi, pi]。
 * - 返回的姿态包含其在输入中的原始索引。
 *
 * @param poses 带索引的输入姿态列表
 * @param startIndex 可选的起始索引（默认为 0）
 * @param positionWeight XYZ 的权重（默认为 1.0）
 * @param orientationWeight RX/RY/RZ 的权重（默认为 0.0 -> 忽略旋转）
 * @return 保留原始索引的重新排序列表
 */
std::vector<PoseWithIndex> sortPosesWithIndex(
    const std::vector<PoseWithIndex>& poses,
    std::size_t startIndex = 0,
    double positionWeight = 1.0,
    double orientationWeight = 0.0);

}  // namespace trajsort

#endif  // POSE_SORT_H