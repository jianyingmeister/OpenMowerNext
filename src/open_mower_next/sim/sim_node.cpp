// sim_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <cmath>
#include <string>
#include <sys/types.h>
#include <sys/stat.h>

using namespace std::chrono_literals;

class SimNode : public rclcpp::Node {
public:
  SimNode() : Node("sim_node") {
    this->declare_parameter<std::string>("serial_device", "/tmp/ublox_sim_write");
    this->declare_parameter<bool>("create_pty", true);
    this->declare_parameter<double>("freq", 10.0);
    this->declare_parameter<double>("init_lat", 37.0);
    this->declare_parameter<double>("init_lon", -122.0);
    this->declare_parameter<double>("init_alt", 10.0);

    serial_device_ = this->get_parameter("serial_device").as_string();
    create_pty_ = this->get_parameter("create_pty").as_bool();
    double freq = this->get_parameter("freq").as_double();
    lat_ = this->get_parameter("init_lat").as_double();
    lon_ = this->get_parameter("init_lon").as_double();
    alt_ = this->get_parameter("init_alt").as_double();

    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
    gps_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/gps/fix", 10);
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(1000.0 / freq)),
                                     std::bind(&SimNode::tick, this));

    master_fd_ = -1;
    slave_fd_ = -1;

    if (create_pty_) {
      std::string slave;
      if (create_pty_pair(master_fd_, slave_fd_, slave)) {
        RCLCPP_INFO(this->get_logger(), "Created PTY pair. slave device: %s", slave.c_str());
        // publish as param for other nodes / convenience
        this->set_parameter(rclcpp::Parameter("pty_slave", slave));
        // also set serial_device_ to master for internal use
        serial_device_ = "<pty_master_fd>";
      } else {
        RCLCPP_WARN(this->get_logger(), "Failed to create PTY pair. Falling back to serial_device param.");
        open_serial_direct();
      }
    } else {
      open_serial_direct();
    }
  }

  ~SimNode() {
    if (master_fd_ > 0) close(master_fd_);
    if (slave_fd_ > 0) close(slave_fd_);
  }

private:
  bool create_pty_pair(int &master_fd, int &slave_fd, std::string &slave_path) {
    // open master pseudo terminal
    master_fd = posix_openpt(O_RDWR | O_NOCTTY);
    if (master_fd < 0) {
      RCLCPP_ERROR(this->get_logger(), "posix_openpt failed: %s", strerror(errno));
      return false;
    }
    if (grantpt(master_fd) != 0) {
      RCLCPP_ERROR(this->get_logger(), "grantpt failed: %s", strerror(errno));
      close(master_fd);
      master_fd = -1;
      return false;
    }
    if (unlockpt(master_fd) != 0) {
      RCLCPP_ERROR(this->get_logger(), "unlockpt failed: %s", strerror(errno));
      close(master_fd);
      master_fd = -1;
      return false;
    }
    char *ptsname_c = ptsname(master_fd);
    if (!ptsname_c) {
      RCLCPP_ERROR(this->get_logger(), "ptsname failed: %s", strerror(errno));
      close(master_fd);
      master_fd = -1;
      return false;
    }
    slave_path = std::string(ptsname_c);

    // open slave side to keep PTY alive; ublox_f9p will also open it for reading
    slave_fd = open(slave_path.c_str(), O_RDWR | O_NOCTTY);
    if (slave_fd < 0) {
      RCLCPP_ERROR(this->get_logger(), "open slave pty %s failed: %s", slave_path.c_str(), strerror(errno));
      close(master_fd);
      master_fd = -1;
      slave_fd = -1;
      return false;
    }

    // configure raw mode for master and slave
    struct termios tio;
    if (tcgetattr(slave_fd, &tio) == 0) {
      cfmakeraw(&tio);
      cfsetispeed(&tio, B115200);
      cfsetospeed(&tio, B115200);
      tcsetattr(slave_fd, TCSANOW, &tio);
    }
    if (tcgetattr(master_fd, &tio) == 0) {
      cfmakeraw(&tio);
      cfsetispeed(&tio, B115200);
      cfsetospeed(&tio, B115200);
      tcsetattr(master_fd, TCSANOW, &tio);
    }

    // keep fds open; master_fd is used for writing NMEA
    return true;
  }

  void open_serial_direct() {
    // For backward compatibility: open given serial device for writing
    if (serial_device_.empty()) {
      RCLCPP_WARN(this->get_logger(), "serial_device param empty");
      return;
    }
    master_fd_ = open(serial_device_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (master_fd_ < 0) {
      RCLCPP_WARN(this->get_logger(), "Could not open serial device %s: %s", serial_device_.c_str(), strerror(errno));
      master_fd_ = -1;
      return;
    }
    struct termios tio;
    tcgetattr(master_fd_, &tio);
    cfmakeraw(&tio);
    cfsetispeed(&tio, B115200);
    cfsetospeed(&tio, B115200);
    tcsetattr(master_fd_, TCSANOW, &tio);
    RCLCPP_INFO(this->get_logger(), "Opened serial device %s", serial_device_.c_str());
  }

  void tick() {
    double dt = 0.1; // approx for 10 Hz
    double east_m = 0.1; // small eastward drift per tick

    // Convert meters to degrees (approx)
    double dlon = (east_m / (111320.0 * std::cos(lat_ * M_PI / 180.0)));
    lon_ += dlon;

    // Publish GPS
    sensor_msgs::msg::NavSatFix gps;
    gps.header.stamp = this->now();
    gps.header.frame_id = "gps_link";
    gps.latitude = lat_;
    gps.longitude = lon_;
    gps.altitude = alt_;
    gps.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    gps.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    gps_pub_->publish(gps);

    // IMU
    sensor_msgs::msg::Imu imu;
    imu.header = gps.header;
    imu.orientation.w = 1.0;
    imu.linear_acceleration.x = 0.0;
    imu.linear_acceleration.y = 0.0;
    imu.linear_acceleration.z = 9.81;
    imu_pub_->publish(imu);

    // Basic odom
    nav_msgs::msg::Odometry odom;
    odom.header = gps.header;
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = 0.0;
    odom.pose.pose.position.y = 0.0;
    odom.pose.pose.position.z = 0.0;
    odom_pub_->publish(odom);

    // TF from base_link to gps_link
    geometry_msgs::msg::TransformStamped t;
    t.header = gps.header;
    t.child_frame_id = "gps_link";
    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.12;
    t.transform.rotation.w = 1.0;
    tf_broadcaster_->sendTransform(t);

    // Write NMEA GGA to master fd if open
    if (master_fd_ > 0) {
      std::string gga = make_nmea_gga(lat_, lon_, alt_);
      ssize_t w = write(master_fd_, gga.c_str(), gga.size());
      if (w < 0) {
        RCLCPP_WARN(this->get_logger(), "Write to serial failed: %s", strerror(errno));
        // try to close and reset master_fd_, will attempt reopen only if direct serial; for pty we keep it
        // but to avoid busy warnings, do nothing here
      }
    }
  }

  std::string make_nmea_gga(double lat, double lon, double alt) {
    auto deg2dm_lat = [](double ang) {
      double a = std::abs(ang);
      double deg = std::floor(a);
      double min = (a - deg) * 60.0;
      char buf[64];
      sprintf(buf, "%02.0f%07.4f", deg, min);
      return std::string(buf);
    };
    auto deg2dm_lon = [](double ang) {
      double a = std::abs(ang);
      double deg = std::floor(a);
      double min = (a - deg) * 60.0;
      char buf[64];
      sprintf(buf, "%03.0f%07.4f", deg, min);
      return std::string(buf);
    };

    std::string lat_str = deg2dm_lat(lat);
    std::string lat_dir = (lat >= 0) ? "N" : "S";
    std::string lon_str = deg2dm_lon(lon);
    std::string lon_dir = (lon >= 0) ? "E" : "W";

    rclcpp::Time tnow = this->now();
    int hh = (int)(tnow.seconds()) / 3600 % 24;
    int mm = (int)(tnow.seconds()) / 60 % 60;
    int ss = (int)(tnow.seconds()) % 60;
    char timestr[16];
    sprintf(timestr, "%02d%02d%02d", hh, mm, ss);

    char body[256];
    sprintf(body, "$GPGGA,%s,%s,%s,%s,%s,1,08,1.0,%.1f,M,0.0,M,,", timestr,
            lat_str.c_str(), lat_dir.c_str(), lon_str.c_str(), lon_dir.c_str(), alt);

    unsigned char csum = 0;
    for (size_t i = 1; i < strlen(body); ++i) csum ^= body[i];
    char chk[8];
    sprintf(chk, "*%02X\r\n", csum);
    std::string out = std::string(body) + std::string(chk);
    return out;
  }

  // members
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::string serial_device_;
  bool create_pty_;
  int master_fd_;
  int slave_fd_;
  double lat_, lon_, alt_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
