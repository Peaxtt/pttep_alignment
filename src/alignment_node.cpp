#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pttep_alignment/srv/calculate_transformation.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <Eigen/Dense>
#include <vector>
#include <cmath>

class AlignmentSolver : public rclcpp::Node {
public:
    AlignmentSolver() : Node("alignment_solver") {
        init_comms();
        RCLCPP_INFO(get_logger(), "== PTTEP WEIGHTED ALIGNMENT SOLVER READY ==");
    }

private:
    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr last_lidar_msg_;
    nav_msgs::msg::Odometry::SharedPtr last_gps_msg_;

    std::vector<Eigen::Vector2d> pts_lidar_, pts_gps_;
    std::vector<double> weights_;

    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_lidar_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_gps_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_save_;
    rclcpp::Service<pttep_alignment::srv::CalculateTransformation>::SharedPtr srv_calc_;

    void init_comms() {
        // 1. LIDAR SLAM (**)
        sub_lidar_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/current_pose", 10, [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
                last_lidar_msg_ = msg;
            });

        // 2. GPS Odometry (***)
        sub_gps_ = create_subscription<nav_msgs::msg::Odometry>(
            "/gps", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                last_gps_msg_ = msg;
            });

        // Service: Save Location
        srv_save_ = create_service<std_srvs::srv::Trigger>("/save_location", 
            std::bind(&AlignmentSolver::on_save_location, this, std::placeholders::_1, std::placeholders::_2));

        // Service: Calculate
        srv_calc_ = create_service<pttep_alignment::srv::CalculateTransformation>("/calculate_transformation",
            std::bind(&AlignmentSolver::on_calculate, this, std::placeholders::_1, std::placeholders::_2));
    }

    // --- Logic 1: Save Location
    void on_save_location(const std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res) {
        // 1.1 เช็ค Data Status
        if (!last_lidar_msg_ || !last_gps_msg_) {
            res->success = false;
            res->message = "Error: Missing sensor data. (Lidar: " + 
                           std::string(last_lidar_msg_ ? "OK" : "No Data") + 
                           ", GPS: " + std::string(last_gps_msg_ ? "OK" : "No Data") + ")";
            RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
            return;
        }

        // 1.2 เช็ค Time Difference
        rclcpp::Time t_lidar(last_lidar_msg_->header.stamp);
        rclcpp::Time t_gps(last_gps_msg_->header.stamp);
        double time_diff = std::abs((t_lidar - t_gps).seconds());

        if (time_diff > 1.0) {
            res->success = false;
            res->message = "Error: Time sync failed. Diff: " + std::to_string(time_diff) + "s (> 1.0s)";
            RCLCPP_WARN(get_logger(), "%s", res->message.c_str());
            return;
        }

        // 1.3 คำนวณน้ำหนักจาก Covariance 
        // ดึง Variance X (index 0) และ Y (index 7) ของ LIDAR
        double var_x = last_lidar_msg_->pose.pose.covariance[0];
        double var_y = last_lidar_msg_->pose.pose.covariance[7];
        double w = 1.0 / (var_x + var_y + 1e-6); 

        pts_lidar_.push_back({last_lidar_msg_->pose.pose.position.x, last_lidar_msg_->pose.pose.position.y});
        pts_gps_.push_back({last_gps_msg_->pose.pose.position.x, last_gps_msg_->pose.pose.position.y});
        weights_.push_back(w);

        res->success = true;
        res->message = "Point #" + std::to_string(pts_lidar_.size()) + " saved. (Diff: " + std::to_string(time_diff) + "s, Weight: " + std::to_string(w) + ")";
        RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
    }

    // --- Logic 2: Calculate, Reset and Frame Mapping ---
    void on_calculate(const pttep_alignment::srv::CalculateTransformation::Request::SharedPtr req,
                      pttep_alignment::srv::CalculateTransformation::Response::SharedPtr res) {
        if (req->reset) {
            pts_lidar_.clear();
            pts_gps_.clear();
            weights_.clear();
            res->success = true;
            res->message = "Memory reset successfully.";
            RCLCPP_INFO(get_logger(), "Memory Cleared.");
            return;
        }

        if (pts_lidar_.size() < 3) {
            res->success = false;
            res->message = "Error: Need at least 3 points for SVD.";
            return;
        }

        // คำนวณด้วยระบบ Weighted SVD
        auto [R, t] = compute_weighted_svd();

        // ตั้งค่า Frame ตาม Requirement (GPS = Header, SLAM = Child)
        res->transform.header.stamp = now();
        res->transform.header.frame_id = last_gps_msg_->header.frame_id;
        res->transform.child_frame_id = last_lidar_msg_->header.frame_id;

        res->transform.transform.translation.x = t.x();
        res->transform.transform.translation.y = t.y();
        
        double yaw = std::atan2(R(1,0), R(0,0));
        res->transform.transform.rotation.z = std::sin(yaw / 2.0);
        res->transform.transform.rotation.w = std::cos(yaw / 2.0);

        res->success = true;
        res->message = "Weighted Alignment successful.";
        
        printf("\n--- FINAL WEIGHTED MATRIX (%s -> %s) ---\n", 
               res->transform.header.frame_id.c_str(), res->transform.child_frame_id.c_str());
        printf("[ %.4f, %.4f, %.4f ]\n", R(0,0), R(0,1), t.x());
        printf("[ %.4f, %.4f, %.4f ]\n", R(1,0), R(1,1), t.y());
        printf("[ 0.0000, 0.0000, 1.0000 ]\n\n");
    }

    // --- Math Engine: Weighted SVD Solver ---
    std::pair<Eigen::Matrix2d, Eigen::Vector2d> compute_weighted_svd() {
        int n = pts_lidar_.size();
        double sum_w = 0;
        Eigen::Vector2d c_l(0,0), c_g(0,0);

        // 1. Weighted Centroids
        for(int i=0; i<n; i++) {
            c_l += weights_[i] * pts_lidar_[i];
            c_g += weights_[i] * pts_gps_[i];
            sum_w += weights_[i];
        }
        c_l /= sum_w; c_g /= sum_w;

        // 2. Weighted Covariance Matrix H
        Eigen::Matrix2d H = Eigen::Matrix2d::Zero();
        for(int i=0; i<n; i++) {
            H += weights_[i] * (pts_lidar_[i] - c_l) * (pts_gps_[i] - c_g).transpose();
        }

        // 3. SVD Calculation
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix2d R = svd.matrixV() * svd.matrixU().transpose();
        
        // Reflection Handling
        if (R.determinant() < 0) {
            Eigen::Matrix2d V = svd.matrixV(); V.col(1) *= -1;
            R = V * svd.matrixU().transpose();
        }
        
        Eigen::Vector2d t = c_g - R * c_l;
        return {R, t};
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AlignmentSolver>());
    rclcpp::shutdown();
    return 0;
}
