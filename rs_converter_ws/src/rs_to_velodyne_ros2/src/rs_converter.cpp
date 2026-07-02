#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include <cmath>

static int RING_ID_MAP_RUBY[] = {
        3, 66, 33, 96, 11, 74, 41, 104, 19, 82, 49, 112, 27, 90, 57, 120,
        35, 98, 1, 64, 43, 106, 9, 72, 51, 114, 17, 80, 59, 122, 25, 88,
        67, 34, 97, 0, 75, 42, 105, 8, 83, 50, 113, 16, 91, 58, 121, 24,
        99, 2, 65, 32, 107, 10, 73, 40, 115, 18, 81, 48, 123, 26, 89, 56,
        7, 70, 37, 100, 15, 78, 45, 108, 23, 86, 53, 116, 31, 94, 61, 124,
        39, 102, 5, 68, 47, 110, 13, 76, 55, 118, 21, 84, 63, 126, 29, 92,
        71, 38, 101, 4, 79, 46, 109, 12, 87, 54, 117, 20, 95, 62, 125, 28,
        103, 6, 69, 36, 111, 14, 77, 44, 119, 22, 85, 52, 127, 30, 93, 60
};

static int RING_ID_MAP_BPEARL[32] = {
     0, 16,
     1, 17,
     2, 18,
     3, 19,
     4, 20,
     5, 21,
     6, 22,
     7, 23,
     8, 24,
     9, 25,
    10, 26,
    11, 27,
    12, 28,
    13, 29,
    14, 30,
    15, 31
};

static int RING_ID_MAP_16[] = {
        0, 1, 2, 3, 4, 5, 6, 7, 15, 14, 13, 12, 11, 10, 9, 8
};

struct RsPointXYZIRT {
    PCL_ADD_POINT4D;
    float intensity;
    uint16_t ring = 0;
    double timestamp = 0;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(RsPointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
    (uint16_t, ring, ring)(double, timestamp, timestamp))

struct VelodynePointXYZIRT {
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(VelodynePointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
    (uint16_t, ring, ring)(float, time, time))

struct VelodynePointXYZIR {
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    uint16_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(VelodynePointXYZIR,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
    (uint16_t, ring, ring))

class RsConverter : public rclcpp::Node {
public:
    RsConverter(const std::string& input_type, const std::string& output_type) 
        : Node("rs_converter"), output_type_(output_type) {
        
        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/velodyne_points", 10);

        if (input_type == "XYZI") {
            sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/rslidar_points", 10,
                std::bind(&RsConverter::rsHandler_XYZI, this, std::placeholders::_1));
        } else if (input_type == "XYZIRT") {
            sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "/rslidar_points", 10,
                std::bind(&RsConverter::rsHandler_XYZIRT, this, std::placeholders::_1));
        } else {
            RCLCPP_ERROR(this->get_logger(), "Unsupported input type: %s", input_type.c_str());
            rclcpp::shutdown();
        }
    }

private:
    template<typename T>
    bool has_nan(T point) {
        if (std::isnan(point.x) || std::isnan(point.y) || std::isnan(point.z)) {
            return true;
        }
        return false;
    }

    template<typename T>
    void publish_points(T& new_pc, const sensor_msgs::msg::PointCloud2& old_msg) {
        new_pc->is_dense = true;
        sensor_msgs::msg::PointCloud2 pc_new_msg;
        pcl::toROSMsg(*new_pc, pc_new_msg);
        pc_new_msg.header = old_msg.header;
        pc_new_msg.header.frame_id = "velodyne";
        pub_->publish(pc_new_msg);
    }

    void rsHandler_XYZI(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr pc(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*msg, *pc);

        size_t num_points = pc->points.size();
        auto pc_new = std::make_shared<pcl::PointCloud<VelodynePointXYZIR>>();
        pc_new->points.reserve(num_points);
        for (size_t point_id = 0; point_id < num_points; ++point_id) {
            if (has_nan(pc->points[point_id])) continue;

            VelodynePointXYZIR new_point;
            new_point.x = pc->points[point_id].x;
            new_point.y = pc->points[point_id].y;
            new_point.z = pc->points[point_id].z;
            new_point.intensity = pc->points[point_id].intensity;

            if (pc->height == 16) {
                new_point.ring = RING_ID_MAP_16[point_id / pc->width];
            } else if (pc->height == 128) {
                new_point.ring = RING_ID_MAP_RUBY[point_id % pc->height];
            } else if (pc->height == 32)  {
                new_point.ring = RING_ID_MAP_BPEARL[point_id % pc->height];
            }

            pc_new->points.push_back(new_point);
        }
        publish_points(pc_new, *msg);
    }

    void rsHandler_XYZIRT(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        auto pc_in = std::make_shared<pcl::PointCloud<RsPointXYZIRT>>();
        pcl::fromROSMsg(*msg, *pc_in);

        size_t num_points = pc_in->points.size();
        if (num_points == 0) return;

        // Pre-compute t0 once outside the loop
        double t0 = pc_in->points[0].timestamp;

        if (output_type_ == "XYZIRT") {
            auto pc_out = std::make_shared<pcl::PointCloud<VelodynePointXYZIRT>>();
            pc_out->points.reserve(num_points);
            for (size_t i = 0; i < num_points; ++i) {
                const auto& p = pc_in->points[i];
                if (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z)) continue;
                VelodynePointXYZIRT new_point;
                new_point.x = p.x;
                new_point.y = p.y;
                new_point.z = p.z;
                new_point.intensity = p.intensity;
                new_point.ring = p.ring;
                new_point.time = static_cast<float>(p.timestamp - t0);
                pc_out->points.push_back(new_point);
            }
            publish_points(pc_out, *msg);
        } else if (output_type_ == "XYZIR") {
            auto pc_out = std::make_shared<pcl::PointCloud<VelodynePointXYZIR>>();
            pc_out->points.reserve(num_points);
            for (size_t i = 0; i < num_points; ++i) {
                const auto& p = pc_in->points[i];
                if (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z)) continue;
                VelodynePointXYZIR new_point;
                new_point.x = p.x;
                new_point.y = p.y;
                new_point.z = p.z;
                new_point.intensity = p.intensity;
                new_point.ring = p.ring;
                pc_out->points.push_back(new_point);
            }
            publish_points(pc_out, *msg);
        } else if (output_type_ == "XYZI") {
            auto pc_out = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
            pc_out->points.reserve(num_points);
            for (size_t i = 0; i < num_points; ++i) {
                const auto& p = pc_in->points[i];
                if (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z)) continue;
                pcl::PointXYZI new_point;
                new_point.x = p.x;
                new_point.y = p.y;
                new_point.z = p.z;
                new_point.intensity = p.intensity;
                pc_out->points.push_back(new_point);
            }
            publish_points(pc_out, *msg);
        }
    }

    std::string output_type_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    if (argc < 3) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Usage: ros2 run rs_converter input_type(XYZI/XYZIRT) output_type(XYZI/XYZIR/XYZIRT)");
        return 1;
    }
    auto node = std::make_shared<RsConverter>(argv[1], argv[2]);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
