/*
 * serial_bridge_node.cpp  —  v1 (ไม่มีการเปลี่ยนแปลง)
 * ─────────────────────────────────────────────────────────────────────────────
 * Subscribe:  /wheel_cmd  (Float32MultiArray [omegaL, omegaR])
 *             → "CMD:omegaL,omegaR\n" → /dev/ttyACM0
 *
 * Publish:    /motor_state  (Float32MultiArray [rpmL, rpmR, pwmL, pwmR])
 *             /imu_raw      (Float32MultiArray [ax, ay, az, gx, gy, gz])
 *
 * ESP32 serial protocol:
 *   RX: CMD:omegaL,omegaR\n
 *   TX: STATE:rpmL,rpmR,pwmL,pwmR\n
 *       IMU:ax,ay,az,gx,gy,gz\n
 *       INFO:...\n
 * ─────────────────────────────────────────────────────────────────────────────
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <errno.h>
#include <sys/select.h>

#include <thread>
#include <mutex>
#include <atomic>
#include <string>

using namespace std::chrono_literals;

class SerialBridgeNode : public rclcpp::Node
{
public:
  SerialBridgeNode()
  : Node("serial_bridge"), fd_(-1), running_(false)
  {
    this->declare_parameter<std::string>("port", "/dev/ttyACM0");
    this->declare_parameter<int>("baud", 115200);
    port_ = this->get_parameter("port").as_string();
    baud_ = this->get_parameter("baud").as_int();

    motor_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/motor_state", 10);
    imu_pub_   = this->create_publisher<std_msgs::msg::Float32MultiArray>("/imu_raw", 50);

    wheel_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/wheel_cmd", 10,
      [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() < 2) return;
        char buf[64];
        int n = snprintf(buf, sizeof(buf), "CMD:%.5f,%.5f\n",
                         (double)msg->data[0], (double)msg->data[1]);
        serialWrite(buf, n);
      });

    if (!openPort()) {
      RCLCPP_ERROR(get_logger(), "Cannot open %s — retrying every 2s", port_.c_str());
      retry_timer_ = this->create_wall_timer(2s, [this]() {
        if (openPort()) {
          retry_timer_->cancel();
          startReadThread();
        }
      });
    } else {
      startReadThread();
    }

    RCLCPP_INFO(get_logger(), "SerialBridge ready: port=%s baud=%d", port_.c_str(), baud_);
  }

  ~SerialBridgeNode() {
    running_ = false;
    if (read_thread_.joinable()) read_thread_.join();
    if (fd_ >= 0) { close(fd_); fd_ = -1; }
  }

private:
  bool openPort() {
    fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) return false;

    struct termios tty{};
    tcgetattr(fd_, &tty);

    speed_t brate = B115200;
    if      (baud_ == 9600)   brate = B9600;
    else if (baud_ == 57600)  brate = B57600;
    else if (baud_ == 230400) brate = B230400;

    cfsetispeed(&tty, brate);
    cfsetospeed(&tty, brate);
    cfmakeraw(&tty);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
      close(fd_); fd_ = -1; return false;
    }
    tcflush(fd_, TCIOFLUSH);

    RCLCPP_INFO(get_logger(), "Opened %s at %d baud", port_.c_str(), baud_);
    return true;
  }

  void serialWrite(const char *buf, int len) {
    if (fd_ < 0) return;
    std::lock_guard<std::mutex> lk(write_mutex_);
    int written = 0;
    while (written < len) {
      int r = write(fd_, buf + written, len - written);
      if (r < 0) break;
      written += r;
    }
  }

  void startReadThread() {
    running_ = true;
    read_thread_ = std::thread([this]() { readLoop(); });
  }

  void readLoop() {
    char raw[512];
    std::string linebuf;
    linebuf.reserve(256);

    while (running_) {
      fd_set fds;
      FD_ZERO(&fds); FD_SET(fd_, &fds);
      struct timeval tv{0, 100000};
      if (select(fd_ + 1, &fds, nullptr, nullptr, &tv) <= 0) continue;

      int n = read(fd_, raw, sizeof(raw));
      if (n <= 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
          RCLCPP_ERROR(get_logger(), "Serial read error — port disconnected?");
          running_ = false;
        }
        continue;
      }

      for (int i = 0; i < n; i++) {
        char c = raw[i];
        if (c == '\n' || c == '\r') {
          if (!linebuf.empty()) {
            parseLine(linebuf);
            linebuf.clear();
          }
        } else {
          if (linebuf.size() < 512) linebuf += c;
          else linebuf.clear();
        }
      }
    }
  }

  void parseLine(const std::string &line) {
    if (line.size() < 4) return;

    if (line.compare(0, 6, "STATE:") == 0) {
      float rpmL, rpmR, pwmL, pwmR;
      if (sscanf(line.c_str() + 6, "%f,%f,%f,%f",
                 &rpmL, &rpmR, &pwmL, &pwmR) == 4) {
        std_msgs::msg::Float32MultiArray msg;
        msg.data = {rpmL, rpmR, pwmL, pwmR};
        motor_pub_->publish(msg);
      }
      return;
    }

    if (line.compare(0, 4, "IMU:") == 0) {
      float v[6] = {};
      if (sscanf(line.c_str() + 4, "%f,%f,%f,%f,%f,%f",
                 &v[0], &v[1], &v[2], &v[3], &v[4], &v[5]) == 6) {
        std_msgs::msg::Float32MultiArray msg;
        msg.data = {v[0], v[1], v[2], v[3], v[4], v[5]};
        imu_pub_->publish(msg);
      }
      return;
    }

    if (line.compare(0, 5, "INFO:") == 0) {
      RCLCPP_INFO(get_logger(), "[ESP32] %s", line.c_str() + 5);
      return;
    }
  }

  int         fd_;
  std::string port_;
  int         baud_;

  std::thread       read_thread_;
  std::atomic<bool> running_;
  std::mutex        write_mutex_;

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr    motor_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr    imu_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr wheel_sub_;
  rclcpp::TimerBase::SharedPtr retry_timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialBridgeNode>());
  rclcpp::shutdown();
  return 0;
}