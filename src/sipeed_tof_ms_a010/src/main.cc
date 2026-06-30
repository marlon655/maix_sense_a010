#include <cv_bridge/cv_bridge.hpp>
#include <memory>
#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <thread>
#include <vector>

#include "cJSON.h"
#include "frame_struct.h"
#include "serial.hh"

extern frame_t *handle_process(std::string s);
extern void handle_process_reset();

using namespace std::chrono_literals;

class SipeedTOF_MSA010_Publisher : public rclcpp::Node {
#define ser (*pser)
 private:
  std::unique_ptr<Serial> pser;
  std::string device_path_;
  float uvf_parms[4];
  double watchdog_timeout_sec_ = 3.0;
  double watchdog_cooldown_sec_ = 1.0;
  rclcpp::Time last_cloud_publish_time_;
  rclcpp::Time last_watchdog_recovery_time_;
  bool recovering_stream_ = false;
  bool stream_active_after_reopen_ = false;

 public:
  SipeedTOF_MSA010_Publisher() : Node("sipeed_tof_ms_a010") {
    std::string s;
    this->declare_parameter("device", "/dev/tof");
    this->declare_parameter("watchdog_timeout_sec", 3.0);
    this->declare_parameter("watchdog_cooldown_sec", 1.0);
    rclcpp::Parameter device_param = this->get_parameter("device");
    s = device_param.as_string();
    device_path_ = s;
    watchdog_timeout_sec_ = this->get_parameter("watchdog_timeout_sec").as_double();
    watchdog_cooldown_sec_ = this->get_parameter("watchdog_cooldown_sec").as_double();
    pser = std::make_unique<Serial>(s);
    std::cout << "use device: " << s << std::endl;
    if (!pser || !pser->ok()) {
      std::cerr << "Serial device is not available: " << s << std::endl;
      return;
    }

    // Experimental startup: do not toggle AT+ISP. Some A010 units can get stuck
    // with "ISP is busy" / "Dragonfly ISP stop failed" after ISP toggling.
    // If the previous process died while streaming, stop UART frame output first
    // and drain a few pending binary chunks before sending AT commands.
    stop_stream_and_drain();
    if (!rclcpp::ok()) {
      return;
    }

    if (!send_command_expect_with_retries("AT", "OK", s, 4, 5)) {
      std::cout << "finish: AT " << s << std::endl;
      return;
    }
    std::cout << "finish: AT " << s << std::endl;
    if (!rclcpp::ok()) {
      return;
    }

    if (!send_command_expect_with_retries("AT+COEFF?", "{", s, 4, 8)) {
      std::cout << "finish: AT+COEFF? " << s << std::endl;
      std::cout << "AT+COEFF error" << std::endl;
      return;
    }
    std::cout << "finish: AT+COEFF? " << s << std::endl;
    if (!s.compare("+COEFF=1\r\nOK\r\n")) {
      s = s.substr(14, s.length() - 14);
      std::cout << "s.substr" << std::endl;
      if (s.length() == 0) {
        ser >> s;
      }
    } else {
      size_t json_start = s.find("{");
      if (json_start != std::string::npos) {
        s = s.substr(json_start);
      } else {
        // not this serial port
        std::cout << "AT+COEFF error" << std::endl;
        return;
      }
    }

    if (s.find("{") == std::string::npos) {
      std::cout << "AT+COEFF error" << std::endl;
      return;
    }

    std::cout << "parse json" << std::endl;
    // cout << s << endl;
    cJSON *cparms = cJSON_ParseWithLength((const char *)s.c_str(), s.length());
    // parse intrinsic params into uvf_parms (fx, fy, u0, v0)
    uvf_parms[0] =
        ((float)((cJSON_GetObjectItem(cparms, "fx")->valueint) / 262144.0f));
    uvf_parms[1] =
        ((float)((cJSON_GetObjectItem(cparms, "fy")->valueint) / 262144.0f));
    uvf_parms[2] =
        ((float)((cJSON_GetObjectItem(cparms, "u0")->valueint) / 262144.0f));
    uvf_parms[3] =
        ((float)((cJSON_GetObjectItem(cparms, "v0")->valueint) / 262144.0f));
    std::cout << "fx: " << uvf_parms[0] << std::endl;
    std::cout << "fy: " << uvf_parms[1] << std::endl;
    std::cout << "u0: " << uvf_parms[2] << std::endl;
    std::cout << "v0: " << uvf_parms[3] << std::endl;

    drain_serial("before stream start", 3);

    if (!send_command_expect_with_retries("AT+DISP=3", "OK", s, 3, 5)) {
      std::cout << "finish: AT+DISP=3 " << s << std::endl;
      return;
    }
    handle_process_reset();
    if (!rclcpp::ok()) {
      return;
    }

    publisher_depth =
        this->create_publisher<sensor_msgs::msg::Image>("depth", 10);
    publisher_pointcloud =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", 10);
    last_cloud_publish_time_ = this->get_clock()->now();
    last_watchdog_recovery_time_ = last_cloud_publish_time_;
    timer_ = this->create_wall_timer(
        30ms, std::bind(&SipeedTOF_MSA010_Publisher::timer_callback, this));
  }

  ~SipeedTOF_MSA010_Publisher() {
    if (timer_) {
      timer_.reset();
    }
    if (pser && pser->ok()) {
      try {
        // Stop UART frame output for the next startup, but do not wait/read here.
        ser << "AT+DISP=1\r";
      } catch (...) {
      }
    }
    pser.reset();
  }



 private:
  bool read_until_contains(const std::string &label, const std::string &expected,
                           std::string &out, int attempts = 5) {
    out.clear();
    if (!pser || !pser->ok()) {
      return false;
    }
    for (int i = 0; i < attempts && rclcpp::ok(); ++i) {
      std::string chunk;
      ser >> chunk;
      if (!chunk.empty()) {
        out += chunk;
        std::cout << label << ": read chunk: " << chunk << std::endl;
        if (out.find(expected) != std::string::npos) {
          return true;
        }
      }
    }
    return false;
  }

  size_t drain_serial(const std::string &label, int attempts = 3) {
    if (!pser || !pser->ok()) {
      return 0;
    }
    int empty_reads = 0;
    size_t drained_bytes = 0;
    for (int i = 0; i < attempts && rclcpp::ok(); ++i) {
      std::string chunk;
      ser >> chunk;
      if (chunk.empty()) {
        if (++empty_reads >= 2) {
          break;
        }
        continue;
      }
      empty_reads = 0;
      drained_bytes += chunk.size();
      std::cout << label << ": drained " << chunk.size() << " bytes" << std::endl;
    }
    return drained_bytes;
  }

  void stop_stream_and_drain() {
    if (!pser || !pser->ok()) {
      return;
    }
    for (int i = 0; i < 3 && rclcpp::ok(); ++i) {
      ser << "AT+DISP=1\r";
      std::this_thread::sleep_for(500ms);
      drain_serial("startup stop stream", 12);
    }
  }

  bool send_command_expect(const std::string &cmd, const std::string &expected,
                           std::string &response, int attempts = 5) {
    if (!pser || !pser->ok() || !rclcpp::ok()) {
      return false;
    }
    ser << cmd + "\r";
    return read_until_contains("finish: " + cmd, expected, response, attempts);
  }

  bool send_command_expect_with_retries(const std::string &cmd,
                                        const std::string &expected,
                                        std::string &response,
                                        int retries,
                                        int attempts_per_retry) {
    for (int i = 0; i < retries && rclcpp::ok(); ++i) {
      if (send_command_expect(cmd, expected, response, attempts_per_retry)) {
        return true;
      }
      std::cout << cmd << ": retry " << (i + 1) << " response: " << response << std::endl;
      drain_serial(cmd + " retry drain", 4);
      std::this_thread::sleep_for(300ms);
    }
    return false;
  }

  std::vector<std::string> reconnect_candidates() const {
    std::vector<std::string> candidates;
    auto add_unique = [&candidates](const std::string &path) {
      for (const auto &candidate : candidates) {
        if (candidate == path) {
          return;
        }
      }
      candidates.push_back(path);
    };

    if (device_path_ != "/dev/tof") {
      add_unique(device_path_);
    }
    add_unique("/dev/ttyUSB0");
    add_unique("/dev/ttyUSB1");
    add_unique("/dev/ttyUSB2");
    add_unique("/dev/tof");
    return candidates;
  }

  bool reopen_serial_and_probe() {
    pser.reset();
    stream_active_after_reopen_ = false;
    std::this_thread::sleep_for(500ms);

    for (const auto &candidate : reconnect_candidates()) {
      if (!rclcpp::ok()) {
        return false;
      }

      std::cout << "watchdog probe device: " << candidate << std::endl;
      auto probe = std::make_unique<Serial>(candidate);
      if (!probe || !probe->ok()) {
        continue;
      }

      pser = std::move(probe);
      std::string response;
      const size_t drained_bytes = drain_serial("watchdog reconnect drain", 4);
      if (drained_bytes > 512) {
        device_path_ = candidate;
        stream_active_after_reopen_ = true;
        handle_process_reset();
        std::cout << "watchdog using active stream device: " << device_path_
                  << " drained_bytes: " << drained_bytes << std::endl;
        return true;
      }

      if (send_command_expect_with_retries("AT", "OK", response, 2, 4)) {
        device_path_ = candidate;
        std::cout << "watchdog using device: " << device_path_ << std::endl;
        return true;
      }

      std::cout << "watchdog probe failed on " << candidate
                << " response: " << response << std::endl;
      pser.reset();
      std::this_thread::sleep_for(300ms);
    }

    return false;
  }

  bool watchdog_should_recover() {
    if (watchdog_timeout_sec_ <= 0.0 || recovering_stream_) {
      return false;
    }

    const rclcpp::Time now = this->get_clock()->now();
    const double since_publish = (now - last_cloud_publish_time_).seconds();
    const double since_recovery = (now - last_watchdog_recovery_time_).seconds();
    return since_publish > watchdog_timeout_sec_ &&
           since_recovery > watchdog_cooldown_sec_;
  }

  void recover_stream_if_needed() {
    if (!watchdog_should_recover()) {
      return;
    }

    recovering_stream_ = true;
    last_watchdog_recovery_time_ = this->get_clock()->now();
    RCLCPP_WARN(this->get_logger(),
                "No /cloud frames for %.1fs. Restarting MaixSense stream.",
                watchdog_timeout_sec_);

    std::string response;
    handle_process_reset();

    RCLCPP_WARN(this->get_logger(), "Watchdog reopening serial.");
    if (!reopen_serial_and_probe()) {
      RCLCPP_WARN(this->get_logger(),
                  "Watchdog could not reopen a MaixSense serial port.");
      recovering_stream_ = false;
      return;
    }

    if (stream_active_after_reopen_) {
      last_cloud_publish_time_ = this->get_clock()->now();
      recovering_stream_ = false;
      RCLCPP_WARN(this->get_logger(),
                  "MaixSense stream was already active after serial reopen.");
      return;
    }

    if (!send_command_expect_with_retries("AT+DISP=3", "OK", response, 3, 5)) {
      RCLCPP_WARN(this->get_logger(),
                  "Watchdog could not restart stream. Response: '%s'",
                  response.c_str());
      recovering_stream_ = false;
      return;
    }

    handle_process_reset();
    last_cloud_publish_time_ = this->get_clock()->now();
    recovering_stream_ = false;
    RCLCPP_WARN(this->get_logger(), "MaixSense stream restarted by watchdog.");
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_depth;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      publisher_pointcloud;

  void timer_callback() {
    if (!rclcpp::ok()) {
      return;
    }

    recover_stream_if_needed();
    if (recovering_stream_ || !pser || !pser->ok()) {
      return;
    }

    std::string s;
    std::stringstream sstream;
    frame_t *f = nullptr;

    // Do not spin forever inside one timer callback. If the serial stream is
    // noisy or continuous, an unbounded loop prevents Ctrl+C/SIGTERM shutdown.
    for (int attempt = 0; attempt < 12 && rclcpp::ok(); ++attempt) {
      ser >> s;
      if (s.empty()) {
        return;
      }
      f = handle_process(s);
      if (f) {
        break;
      }
    }

    if (!f || !rclcpp::ok()) {
      return;
    }
    // cout << f << endl;
    uint8_t rows, cols, *depth;
    rows = f->frame_head.resolution_rows;
    cols = f->frame_head.resolution_cols;
    depth = f->payload;
    cv::Mat md(rows, cols, CV_8UC1, depth);

    sstream << md.size();

    // ===== DEBUG: print raw depth values at key pixels =====
    // {
    //   uint32_t mid_j = rows / 2;
    //   uint32_t mid_i = cols / 2;
    //   uint8_t raw_center = depth[mid_j * cols + mid_i];
    //   uint8_t raw_00     = depth[0];
    //   uint8_t raw_corner = depth[(rows - 1) * cols + (cols - 1)];

    //   // Also compute min/max/avg over entire frame
    //   uint8_t raw_min = 255, raw_max = 0;
    //   uint32_t raw_sum = 0;
    //   uint32_t nonzero_count = 0;
    //   for (uint32_t jj = 0; jj < rows; ++jj) {
    //     for (uint32_t ii = 0; ii < cols; ++ii) {
    //       uint8_t v = depth[jj * cols + ii];
    //       if (v < raw_min) raw_min = v;
    //       if (v > raw_max) raw_max = v;
    //       raw_sum += v;
    //       if (v > 0) nonzero_count++;
    //     }
    //   }
    //   float raw_avg = (rows * cols > 0) ? static_cast<float>(raw_sum) / (rows * cols) : 0.0f;

    //   RCLCPP_INFO(this->get_logger(),
    //     "DEBUG depth [%ux%u] center(%u,%u)=%u  (0,0)=%u  corner=%u  "
    //     "min=%u max=%u avg=%.1f nonzero=%u",
    //     cols, rows, mid_i, mid_j, raw_center, raw_00, raw_corner,
    //     raw_min, raw_max, raw_avg, nonzero_count);

    //   // Print what each divisor would give for the center pixel
    //   RCLCPP_INFO(this->get_logger(),
    //     "DEBUG center raw=%u => /50=%.4fm  /100=%.4fm  /200=%.4fm  /1000=%.5fm pow(raw_center / 5.1, 2)=%.4fm",
    //     raw_center,
    //     raw_center / 50.0f,
    //     raw_center / 100.0f,
    //     raw_center / 200.0f,
    //     raw_center / 1000.0f,
    //     pow(raw_center / 5.1, 2));
    // }
    // ===== END DEBUG =====

    std_msgs::msg::Header header;
    header.stamp = this->get_clock()->now();
    header.frame_id = "tof";

    // build image message and publish
    auto img_msg = cv_bridge::CvImage(header, "mono8", md).toImageMsg();
    // RCLCPP_INFO(this->get_logger(), "Publishing: depth:%s",
    //             sstream.str().c_str());
    publisher_depth->publish(*img_msg);

    // build pointcloud
    sensor_msgs::msg::PointCloud2 pcmsg;
    pcmsg.header = header;
    pcmsg.height = static_cast<uint32_t>(rows);
    pcmsg.width = static_cast<uint32_t>(cols);
    pcmsg.is_bigendian = false;
    pcmsg.point_step = 16; // 3 floats + 1 uint32 (rgb)
    pcmsg.row_step = pcmsg.point_step * pcmsg.width; // <-- corrected (width)
    pcmsg.is_dense = false;
    pcmsg.fields.resize(4); // explicit 4 fields

    pcmsg.fields[0].name = "x";
    pcmsg.fields[0].offset = 0;
    pcmsg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pcmsg.fields[0].count = 1;

    pcmsg.fields[1].name = "y";
    pcmsg.fields[1].offset = 4;
    pcmsg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pcmsg.fields[1].count = 1;

    pcmsg.fields[2].name = "z";
    pcmsg.fields[2].offset = 8;
    pcmsg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    pcmsg.fields[2].count = 1;

    pcmsg.fields[3].name = "rgb";
    pcmsg.fields[3].offset = 12;
    pcmsg.fields[3].datatype = sensor_msgs::msg::PointField::UINT32;
    pcmsg.fields[3].count = 1;

    // intrinsics from parsed COEFF
    float fox = uvf_parms[0];
    float foy = uvf_parms[1];
    float u0 = uvf_parms[2];
    float v0 = uvf_parms[3];

    pcmsg.data.resize((pcmsg.height) * (pcmsg.width) * (pcmsg.point_step),
                      0x00);
    uint8_t *ptr = pcmsg.data.data();

    // iterate with unsigned counters that match pcmsg types
    for (uint32_t j = 0; j < pcmsg.height; ++j) {
      for (uint32_t i = 0; i < pcmsg.width; ++i) {
        float cx = (static_cast<float>(i) - u0) / fox;
        float cy = (static_cast<float>(j) - v0) / foy;
        // float dst = (static_cast<float>(depth[j * (pcmsg.width) + i])) / 110; ///  230.0f; // essa divisao por 230 é feita para normalizar a leitura (por algum motivo ela era feita por 1000)
        float p = static_cast<float>(depth[j * (pcmsg.width) + i]);
        float dst = pow(p / 5.1f, 2.0f) / 1000.0f; // metros
        // o 230 era o valor correto para alguma configuracao q estavamos usando no sipeed antes, o marcos mudou algo e passou a ser ~110. Caso tenha duvidas, meça a distancia da leitura real do sipeed
        // meça a distancia da leitura q ele ta pegando no rivz
        // descomente a parte de debug ali em cima, e veja como os resultados se comparam com oq vc ta vendo
        float x = dst * cx;
        float y = dst * cy;
        float z = dst;

        // write floats safely with memcpy (avoid aliasing/alignment UB)
        memcpy(ptr + 0, &x, sizeof(float));
        memcpy(ptr + 4, &y, sizeof(float));
        memcpy(ptr + 8, &z, sizeof(float));

        const uint8_t *color = color_lut_jet[depth[j * (pcmsg.width) + i]];
        uint32_t rgb = (static_cast<uint32_t>(color[0]) << 16) |
                       (static_cast<uint32_t>(color[1]) << 8) |
                       (static_cast<uint32_t>(color[2]));
        memcpy(ptr + 12, &rgb, sizeof(uint32_t));

        ptr += pcmsg.point_step;
      }
    }
    publisher_pointcloud->publish(pcmsg);
    last_cloud_publish_time_ = this->get_clock()->now();

    free(f);
  }

  const uint8_t color_lut_jet[256][3] = {
      {128, 0, 0},     {132, 0, 0},     {136, 0, 0},     {140, 0, 0},
      {144, 0, 0},     {148, 0, 0},     {152, 0, 0},     {156, 0, 0},
      {160, 0, 0},     {164, 0, 0},     {168, 0, 0},     {172, 0, 0},
      {176, 0, 0},     {180, 0, 0},     {184, 0, 0},     {188, 0, 0},
      {192, 0, 0},     {196, 0, 0},     {200, 0, 0},     {204, 0, 0},
      {208, 0, 0},     {212, 0, 0},     {216, 0, 0},     {220, 0, 0},
      {224, 0, 0},     {228, 0, 0},     {232, 0, 0},     {236, 0, 0},
      {240, 0, 0},     {244, 0, 0},     {248, 0, 0},     {252, 0, 0},
      {255, 0, 0},     {255, 4, 0},     {255, 8, 0},     {255, 12, 0},
      {255, 16, 0},    {255, 20, 0},    {255, 24, 0},    {255, 28, 0},
      {255, 32, 0},    {255, 36, 0},    {255, 40, 0},    {255, 44, 0},
      {255, 48, 0},    {255, 52, 0},    {255, 56, 0},    {255, 60, 0},
      {255, 64, 0},    {255, 68, 0},    {255, 72, 0},    {255, 76, 0},
      {255, 80, 0},    {255, 84, 0},    {255, 88, 0},    {255, 92, 0},
      {255, 96, 0},    {255, 100, 0},   {255, 104, 0},   {255, 108, 0},
      {255, 112, 0},   {255, 116, 0},   {255, 120, 0},   {255, 124, 0},
      {255, 128, 0},   {255, 132, 0},   {255, 136, 0},   {255, 140, 0},
      {255, 144, 0},   {255, 148, 0},   {255, 152, 0},   {255, 156, 0},
      {255, 160, 0},   {255, 164, 0},   {255, 168, 0},   {255, 172, 0},
      {255, 176, 0},   {255, 180, 0},   {255, 184, 0},   {255, 188, 0},
      {255, 192, 0},   {255, 196, 0},   {255, 200, 0},   {255, 204, 0},
      {255, 208, 0},   {255, 212, 0},   {255, 216, 0},   {255, 220, 0},
      {255, 224, 0},   {255, 228, 0},   {255, 232, 0},   {255, 236, 0},
      {255, 240, 0},   {255, 244, 0},   {255, 248, 0},   {255, 252, 0},
      {254, 255, 1},   {250, 255, 6},   {246, 255, 10},  {242, 255, 14},
      {238, 255, 18},  {234, 255, 22},  {230, 255, 26},  {226, 255, 30},
      {222, 255, 34},  {218, 255, 38},  {214, 255, 42},  {210, 255, 46},
      {206, 255, 50},  {202, 255, 54},  {198, 255, 58},  {194, 255, 62},
      {190, 255, 66},  {186, 255, 70},  {182, 255, 74},  {178, 255, 78},
      {174, 255, 82},  {170, 255, 86},  {166, 255, 90},  {162, 255, 94},
      {158, 255, 98},  {154, 255, 102}, {150, 255, 106}, {146, 255, 110},
      {142, 255, 114}, {138, 255, 118}, {134, 255, 122}, {130, 255, 126},
      {126, 255, 130}, {122, 255, 134}, {118, 255, 138}, {114, 255, 142},
      {110, 255, 146}, {106, 255, 150}, {102, 255, 154}, {98, 255, 158},
      {94, 255, 162},  {90, 255, 166},  {86, 255, 170},  {82, 255, 174},
      {78, 255, 178},  {74, 255, 182},  {70, 255, 186},  {66, 255, 190},
      {62, 255, 194},  {58, 255, 198},  {54, 255, 202},  {50, 255, 206},
      {46, 255, 210},  {42, 255, 214},  {38, 255, 218},  {34, 255, 222},
      {30, 255, 226},  {26, 255, 230},  {22, 255, 234},  {18, 255, 238},
      {14, 255, 242},  {10, 255, 246},  {6, 255, 250},   {2, 255, 254},
      {0, 252, 255},   {0, 248, 255},   {0, 244, 255},   {0, 240, 255},
      {0, 236, 255},   {0, 232, 255},   {0, 228, 255},   {0, 224, 255},
      {0, 220, 255},   {0, 216, 255},   {0, 212, 255},   {0, 208, 255},
      {0, 204, 255},   {0, 200, 255},   {0, 196, 255},   {0, 192, 255},
      {0, 188, 255},   {0, 184, 255},   {0, 180, 255},   {0, 176, 255},
      {0, 172, 255},   {0, 168, 255},   {0, 164, 255},   {0, 160, 255},
      {0, 156, 255},   {0, 152, 255},   {0, 148, 255},   {0, 144, 255},
      {0, 140, 255},   {0, 136, 255},   {0, 132, 255},   {0, 128, 255},
      {0, 124, 255},   {0, 120, 255},   {0, 116, 255},   {0, 112, 255},
      {0, 108, 255},   {0, 104, 255},   {0, 100, 255},   {0, 96, 255},
      {0, 92, 255},    {0, 88, 255},    {0, 84, 255},    {0, 80, 255},
      {0, 76, 255},    {0, 72, 255},    {0, 68, 255},    {0, 64, 255},
      {0, 60, 255},    {0, 56, 255},    {0, 52, 255},    {0, 48, 255},
      {0, 44, 255},    {0, 40, 255},    {0, 36, 255},    {0, 32, 255},
      {0, 28, 255},    {0, 24, 255},    {0, 20, 255},    {0, 16, 255},
      {0, 12, 255},    {0, 8, 255},     {0, 4, 255},     {0, 0, 255},
      {0, 0, 252},     {0, 0, 248},     {0, 0, 244},     {0, 0, 240},
      {0, 0, 236},     {0, 0, 232},     {0, 0, 228},     {0, 0, 224},
      {0, 0, 220},     {0, 0, 216},     {0, 0, 212},     {0, 0, 208},
      {0, 0, 204},     {0, 0, 200},     {0, 0, 196},     {0, 0, 192},
      {0, 0, 188},     {0, 0, 184},     {0, 0, 180},     {0, 0, 176},
      {0, 0, 172},     {0, 0, 168},     {0, 0, 164},     {0, 0, 160},
      {0, 0, 156},     {0, 0, 152},     {0, 0, 148},     {0, 0, 144},
      {0, 0, 140},     {0, 0, 136},     {0, 0, 132},     {0, 0, 128}};
};

int main(int argc, char const *argv[]) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<SipeedTOF_MSA010_Publisher>();
    if (rclcpp::ok()) {
      rclcpp::spin(node);
    }
  } catch (const std::exception & e) {
    std::cerr << "sipeed_tof_node exception: " << e.what() << std::endl;
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }

  return 0;
}
