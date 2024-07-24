#include <vector>
#include <algorithm> // 用于std::max和std::min
#include <sensor_msgs/LaserScan.h> // 包含LaserScan消息类型

class AMCLLaserData {
public:
    AMCLLaserData() : range_count(0), range_max(0.0), angle_min(0.0), angle_max(6.28), angle_increment(0.0) {}

    void update_from_scan(const sensor_msgs::LaserScan& scan) {
        // 使用sensor_msgs::LaserScan类型更新
        range_max = scan.range_max;
        angle_min = scan.angle_min;
        angle_increment = scan.angle_increment;
        angle_max = scan.angle_max;
        range_min = scan.range_min;

        ranges.clear(); // 清空距离列表
        for (auto r : scan.ranges) {
            if (r < range_min) {
                ranges.push_back(0);
            } else if (r > scan.range_max) {
                ranges.push_back(scan.range_max);
            } else {
                ranges.push_back(r);
            }
        }
        range_count = scan.ranges.size();
    }

public:
    int range_count; // 测量点数量
    double range_max; // 最大测量距离
    std::vector<double> ranges; // 距离值列表
    double angle_min; // 最小角度
    double angle_max; // 最大角度
    double angle_increment; // 角度增量
    double range_min; // 最小测量距离
};