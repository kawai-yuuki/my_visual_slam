#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <string>
#include <vector>
#include <filesystem>
#include <algorithm>
#include <fstream>
#include <sstream>

using namespace std::chrono_literals;
namespace fs = std::filesystem;

class EurocPlayerNode : public rclcpp::Node {
public:
  EurocPlayerNode() : Node("euroc_player_node"), current_frame_index_(0), current_imu_index_(0) {
    // パラメータの宣言と取得
    this->declare_parameter<std::string>("dataset_path", "");
    this->declare_parameter<double>("publish_rate", 10.0);

    dataset_path_ = this->get_parameter("dataset_path").as_string();
    double publish_rate = this->get_parameter("publish_rate").as_double();

    if (dataset_path_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Dataset path is not set!");
      return; // パスが空なら何もしない
    }

    RCLCPP_INFO(this->get_logger(), "Loading dataset from: %s", dataset_path_.c_str());

    // 画像ファイル名の収集とソート
    std::string cam0_path =dataset_path_ + "/mav0/cam0/data";
    for (const auto& entry : fs::directory_iterator(cam0_path)) {
      if (entry.is_regular_file()) {
        file_names_.push_back(entry.path().filename().string());
      }
    }
    std::sort(file_names_.begin(), file_names_.end());

    // IMUデータの読み込み
    std::string imu_file_path = dataset_path_ + "/mav0/imu0/data.csv";
    std::ifstream imu_file(imu_file_path);
    if (!imu_file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open IMU data file: %s", imu_file_path.c_str());
      return;
    }
    std::string line;
    // ヘッダー行をスキップ
    std::getline(imu_file, line);
    while (std::getline(imu_file, line)) {
      std::istringstream ss(line);
      std::string token;
      ImuData imu_data;
      std::getline(ss, token, ',');
      imu_data.timestamp = std::stoll(token);
      std::getline(ss, token, ','); imu_data.gyro_x = std::stod(token);
      std::getline(ss, token, ','); imu_data.gyro_y = std::stod(token);
      std::getline(ss, token, ','); imu_data.gyro_z = std::stod(token);
      std::getline(ss, token, ','); imu_data.acc_x = std::stod(token);
      std::getline(ss, token, ','); imu_data.acc_y = std::stod(token);
      std::getline(ss, token, ','); imu_data.acc_z = std::stod(token);
      imu_data_list_.push_back(imu_data);
    }
    imu_file.close();
    RCLCPP_INFO(this->get_logger(), "Loaded %zu IMU measurements", imu_data_list_.size());

    // Publisherの作成（左カメラ: iamge_raw, 右カメラ: image_raw_right）
    pub_left_ = this->create_publisher<sensor_msgs::msg::Image>("camera/left/image_raw", 10);
    pub_right_ = this->create_publisher<sensor_msgs::msg::Image>("camera/right/image_raw", 10);

    // タイマーの設定（一定間隔でtimer_callbackを呼び出す）
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate),
      std::bind(&EurocPlayerNode::timer_callback, this)
    );

    // IMU Publisherとタイマーの設定
    pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 100);

    imu_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / 200.0),
      std::bind(&EurocPlayerNode::imu_timer_callback, this)
    );
  }
private:
  struct ImuData {
    int64_t timestamp;      // 時刻
    double gyro_x, gyro_y, gyro_z; // 角速度
    double acc_x, acc_y, acc_z;    // 加速度
  };

  void timer_callback() {
    // 画像ファイル名の生成
    if (current_frame_index_ >= file_names_.size()) return;
    std::string filename = file_names_[current_frame_index_];

    // パスの構築
    //dataset_path_/image_0/000000.png
    std::string left_image_path = dataset_path_ + "/mav0/cam0/data/" + filename;
    std::string right_image_path = dataset_path_ + "/mav0/cam1/data/" + filename;

    // OpenCVで画像を読み込み
    cv::Mat img_left = cv::imread(left_image_path, cv::IMREAD_GRAYSCALE);
    cv::Mat img_right = cv::imread(right_image_path, cv::IMREAD_GRAYSCALE);

    // 画像が存在しない場合は終了
    if (img_left.empty() || img_right.empty()) {
      RCLCPP_INFO(this->get_logger(), "End of stream or failed to load image: %s", filename.c_str());
      // ループさせたい場合はここでindexをリセットする
      // current_frame_index_ = 0;
      return;
    }

    // ROSメッセージに変換
    std_msgs::msg::Header header;
    header.stamp = this->now();
    header.frame_id = "camera_link";

    sensor_msgs::msg::Image::SharedPtr msg_left = cv_bridge::CvImage(header, "mono8", img_left).toImageMsg();
    sensor_msgs::msg::Image::SharedPtr msg_right = cv_bridge::CvImage(header, "mono8", img_right).toImageMsg();

    // 画像をパブリッシュ
    pub_left_->publish(*msg_left);
    pub_right_->publish(*msg_right);

    RCLCPP_INFO(this->get_logger(), "Published frame: %d", current_frame_index_);
    current_frame_index_++;
  }

  void imu_timer_callback() {
    if (current_imu_index_ >= imu_data_list_.size()) return;

    const ImuData& imu_data = imu_data_list_[current_imu_index_];

    // IMUメッセージの作成
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = this->now();
    imu_msg.header.frame_id = "imu_link";
    imu_msg.angular_velocity.x = imu_data.gyro_x;
    imu_msg.angular_velocity.y = imu_data.gyro_y;
    imu_msg.angular_velocity.z = imu_data.gyro_z;
    imu_msg.linear_acceleration.x = imu_data.acc_x;
    imu_msg.linear_acceleration.y = imu_data.acc_y;
    imu_msg.linear_acceleration.z = imu_data.acc_z;

    // IMUデータをパブリッシュ
    pub_imu_->publish(imu_msg);

    RCLCPP_DEBUG(this->get_logger(), "Published IMU data: %zu", current_imu_index_);
    current_imu_index_++;
  }

  std::string dataset_path_;
  size_t current_frame_index_;
  size_t current_imu_index_;
  std::vector<std::string> file_names_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_left_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_right_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr imu_timer_;
  std::vector<ImuData> imu_data_list_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EurocPlayerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}