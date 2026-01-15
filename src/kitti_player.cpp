#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <string>
#include <vector>
#include <filesystem>

using namespace std::chrono_literals;

class KittiPlayerNode : public rclcpp::Node {
public:
  KittiPlayerNode() : Node("kitti_player_node"), current_frame_index_(0) {
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

    // Publisherの作成（左カメラ: iamge_raw, 右カメラ: image_raw_right）
    pub_left_ = this->create_publisher<sensor_msgs::msg::Image>("camera/left/image_raw", 10);
    pub_right_ = this->create_publisher<sensor_msgs::msg::Image>("camera/right/image_raw", 10);

    // タイマーの設定（一定間隔でtimer_callbackを呼び出す）
    timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate),
      std::bind(&KittiPlayerNode::timer_callback, this)
    );
  }
private:
  void timer_callback() {
    // 画像ファイル名の生成
    // 6桁のゼロパディングされたファイル名を生成
    std::stringstream ss;
    ss << std::setw(6) << std::setfill('0') << current_frame_index_;
    std::string filename = ss.str() + ".png";

    // パスの構築
    //dataset_path_/image_0/000000.png
    std::string left_image_path = dataset_path_ + "/image_0/" + filename;
    std::string right_image_path = dataset_path_ + "/image_1/" + filename;

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

  std::string dataset_path_;
  size_t current_frame_index_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_left_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_right_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<KittiPlayerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
