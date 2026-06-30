#include <chrono>
#include <memory>
#include <string>
#include <sstream>
#include <thread>
#include <vector>
#include <utility>

#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge/cv_bridge.hpp>

using namespace std::chrono_literals;

class CameraCaptureNode : public rclcpp::Node
{
public:
  CameraCaptureNode() : Node("camera_capture_node")
  {
    // =========================================================
    // PARAMETERS
    // =========================================================
    declare_parameter<std::string>("device", "/dev/video0");
    declare_parameter<int>("width", 640);
    declare_parameter<int>("height", 480);

    rcl_interfaces::msg::ParameterDescriptor fps_descriptor;
    fps_descriptor.description = "Camera FPS. Accepts int or float.";
    declare_parameter<double>("fps", 15.0, fps_descriptor);

    declare_parameter<std::string>("image_topic", "/camera/image_raw");
    declare_parameter<std::string>("status_topic", "/camera/status");
    declare_parameter<std::string>("frame_id", "camera_frame");

    // auto / libcamera / v4l2 / custom
    declare_parameter<std::string>("gst_source", "libcamera");

    // Якщо хочеш вручну задати pipeline з launch
    declare_parameter<std::string>("gstreamer_pipeline", "");

    declare_parameter<bool>("publish_status", true);
    declare_parameter<double>("retry_period_sec", 5.0);

    declare_parameter<int>("warmup_read_attempts", 25);
    declare_parameter<double>("warmup_sleep_sec", 0.05);

    declare_parameter<bool>("flip_horizontal", false);
    declare_parameter<bool>("flip_vertical", false);

    // =========================================================
    // READ PARAMETERS
    // =========================================================
    device_ = get_parameter("device").as_string();

    width_ = get_parameter("width").as_int();
    height_ = get_parameter("height").as_int();

    fps_ = get_parameter("fps").as_double();
    if (fps_ <= 0.0) {
      fps_ = 15.0;
    }

    fps_int_ = std::max(1, static_cast<int>(std::round(fps_)));

    image_topic_ = get_parameter("image_topic").as_string();
    status_topic_ = get_parameter("status_topic").as_string();
    frame_id_ = get_parameter("frame_id").as_string();

    gst_source_ = to_lower(trim(get_parameter("gst_source").as_string()));
    custom_pipeline_ = get_parameter("gstreamer_pipeline").as_string();

    publish_status_enabled_ = get_parameter("publish_status").as_bool();
    retry_period_sec_ = get_parameter("retry_period_sec").as_double();

    warmup_read_attempts_ = get_parameter("warmup_read_attempts").as_int();
    warmup_sleep_sec_ = get_parameter("warmup_sleep_sec").as_double();

    flip_horizontal_ = get_parameter("flip_horizontal").as_bool();
    flip_vertical_ = get_parameter("flip_vertical").as_bool();

    // =========================================================
    // STATE
    // =========================================================
    camera_ok_ = false;
    active_pipeline_name_ = "";
    active_pipeline_text_ = "";
    last_error_ = "not_started";

    frame_counter_ = 0;
    failed_reads_ = 0;

    last_retry_time_ = 0.0;
    last_status_time_ = 0.0;

    // =========================================================
    // ROS
    // =========================================================
    image_pub_ = create_publisher<sensor_msgs::msg::Image>(image_topic_, 10);
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic_, 10);

    double timer_period = 1.0 / std::max(fps_, 1.0);
    timer_ = create_wall_timer(
      std::chrono::duration<double>(timer_period),
      std::bind(&CameraCaptureNode::timer_callback, this));

    RCLCPP_INFO(
      get_logger(),
      "\u2705 camera_capture_node started | backend=GSTREAMER, gst_source=%s, "
      "device=%s, size=%dx%d, fps=%.2f, topic=%s",
      gst_source_.c_str(), device_.c_str(), width_, height_, fps_, image_topic_.c_str());

    start_pipeline();
  }

  ~CameraCaptureNode() override
  {
    close_camera();
  }

private:
  // =========================================================
  // HELPERS
  // =========================================================
  static std::string trim(const std::string & s)
  {
    size_t start = s.find_first_not_of(" \t\n\r");
    size_t end = s.find_last_not_of(" \t\n\r");
    if (start == std::string::npos) {
      return "";
    }
    return s.substr(start, end - start + 1);
  }

  static std::string to_lower(const std::string & s)
  {
    std::string out = s;
    std::transform(out.begin(), out.end(), out.begin(), ::tolower);
    return out;
  }

  static double now_sec()
  {
    return std::chrono::duration<double>(
      std::chrono::system_clock::now().time_since_epoch()).count();
  }

  // =========================================================
  // PIPELINES
  // =========================================================
  std::vector<std::pair<std::string, std::string>> build_pipelines()
  {
    std::vector<std::pair<std::string, std::string>> pipelines;

    if (!trim(custom_pipeline_).empty()) {
      pipelines.emplace_back("custom", trim(custom_pipeline_));
    }

    // =========================================================
    // Raspberry Pi Camera / IMX708
    //
    // ВАЖЛИВО:
    // Без format=RGBx/BGRx libcamerasrc може вибрати RAW Bayer:
    // SBGGR/RAW, який OpenCV через appsink нормально не читає.
    // Тому примусово просимо готовий video/x-raw RGBx/BGRx.
    // =========================================================

    std::ostringstream libcamera_rgbx;
    libcamera_rgbx
      << "libcamerasrc ! "
      << "video/x-raw,format=RGBx,width=" << width_
      << ",height=" << height_ << ",framerate=" << fps_int_ << "/1 ! "
      << "queue leaky=downstream max-size-buffers=1 ! "
      << "videoconvert ! "
      << "video/x-raw,format=BGR ! "
      << "appsink drop=true max-buffers=1 sync=false";

    std::ostringstream libcamera_bgrx;
    libcamera_bgrx
      << "libcamerasrc ! "
      << "video/x-raw,format=BGRx,width=" << width_
      << ",height=" << height_ << ",framerate=" << fps_int_ << "/1 ! "
      << "queue leaky=downstream max-size-buffers=1 ! "
      << "videoconvert ! "
      << "video/x-raw,format=BGR ! "
      << "appsink drop=true max-buffers=1 sync=false";

    // Іноді на Raspberry Pi 5 стабільніше заводиться через 1536x864,
    // а вже потім resize в OpenCV/vision. Лишаємо як fallback.
    std::ostringstream libcamera_rgbx_1536;
    libcamera_rgbx_1536
      << "libcamerasrc ! "
      << "video/x-raw,format=RGBx,width=1536,height=864,framerate=" << fps_int_ << "/1 ! "
      << "queue leaky=downstream max-size-buffers=1 ! "
      << "videoscale ! "
      << "video/x-raw,width=" << width_ << ",height=" << height_ << " ! "
      << "videoconvert ! "
      << "video/x-raw,format=BGR ! "
      << "appsink drop=true max-buffers=1 sync=false";

    // Запасний v4l2 pipeline. Для твоєї Pi Camera він, скоріш за все,
    // не буде основним, але нехай буде як fallback.
    std::ostringstream v4l2_raw;
    v4l2_raw
      << "v4l2src device=" << device_ << " ! "
      << "video/x-raw,width=" << width_ << ",height=" << height_
      << ",framerate=" << fps_int_ << "/1 ! "
      << "queue leaky=downstream max-size-buffers=1 ! "
      << "videoconvert ! "
      << "video/x-raw,format=BGR ! "
      << "appsink drop=true max-buffers=1 sync=false";

    std::ostringstream v4l2_mjpg;
    v4l2_mjpg
      << "v4l2src device=" << device_ << " ! "
      << "image/jpeg,width=" << width_ << ",height=" << height_
      << ",framerate=" << fps_int_ << "/1 ! "
      << "jpegdec ! "
      << "videoconvert ! "
      << "video/x-raw,format=BGR ! "
      << "appsink drop=true max-buffers=1 sync=false";

    if (gst_source_ == "custom") {
      return pipelines;
    }

    if (gst_source_ == "libcamera") {
      pipelines.emplace_back("libcamera_rgbx", libcamera_rgbx.str());
      pipelines.emplace_back("libcamera_bgrx", libcamera_bgrx.str());
      pipelines.emplace_back("libcamera_rgbx_1536_scaled", libcamera_rgbx_1536.str());
    } else if (gst_source_ == "v4l2") {
      pipelines.emplace_back("v4l2_raw", v4l2_raw.str());
      pipelines.emplace_back("v4l2_mjpg", v4l2_mjpg.str());
    } else {
      // auto
      pipelines.emplace_back("libcamera_rgbx", libcamera_rgbx.str());
      pipelines.emplace_back("libcamera_bgrx", libcamera_bgrx.str());
      pipelines.emplace_back("libcamera_rgbx_1536_scaled", libcamera_rgbx_1536.str());
      pipelines.emplace_back("v4l2_raw", v4l2_raw.str());
      pipelines.emplace_back("v4l2_mjpg", v4l2_mjpg.str());
    }

    return pipelines;
  }

  // =========================================================
  // START / STOP
  // =========================================================
  bool start_pipeline()
  {
    close_camera();

    auto pipelines = build_pipelines();

    if (pipelines.empty()) {
      camera_ok_ = false;
      last_error_ = "no_pipeline_candidates";
      RCLCPP_ERROR(get_logger(), "\u274c No GStreamer pipeline candidates");
      return false;
    }

    for (const auto & [name, pipeline] : pipelines) {
      RCLCPP_INFO(get_logger(), "\U0001F4F7 Trying GStreamer pipeline: %s", name.c_str());
      RCLCPP_INFO(get_logger(), "%s", pipeline.c_str());

      auto cap = std::make_shared<cv::VideoCapture>();
      try {
        cap->open(pipeline, cv::CAP_GSTREAMER);
      } catch (const cv::Exception & e) {
        RCLCPP_WARN(get_logger(), "\u26A0\uFE0F VideoCapture exception for %s: %s",
          name.c_str(), e.what());
        continue;
      }

      if (!cap->isOpened()) {
        RCLCPP_WARN(get_logger(), "\u26A0\uFE0F Pipeline did not open: %s", name.c_str());
        cap->release();
        continue;
      }

      cv::Mat frame;
      bool got_frame = try_read_warmup(*cap, frame);

      if (!got_frame) {
        RCLCPP_WARN(get_logger(), "\u26A0\uFE0F Pipeline opened but no frame: %s", name.c_str());
        cap->release();
        continue;
      }

      cap_ = cap;
      camera_ok_ = true;

      active_pipeline_name_ = name;
      active_pipeline_text_ = pipeline;

      last_error_ = "ok";
      failed_reads_ = 0;

      RCLCPP_INFO(get_logger(), "\u2705 Camera pipeline started successfully: %s", name.c_str());

      return true;
    }

    camera_ok_ = false;
    active_pipeline_name_ = "";
    active_pipeline_text_ = "";
    last_error_ = "failed_to_start_gstreamer_pipeline";

    RCLCPP_ERROR(get_logger(), "\u274c Failed to start any GStreamer pipeline. Will retry.");

    return false;
  }

  bool try_read_warmup(cv::VideoCapture & cap, cv::Mat & frame_out)
  {
    int attempts = std::max(1, warmup_read_attempts_);
    for (int i = 0; i < attempts; ++i) {
      cv::Mat frame;
      bool ok = cap.read(frame);

      if (ok && !frame.empty()) {
        frame_out = frame;
        return true;
      }

      std::this_thread::sleep_for(std::chrono::duration<double>(warmup_sleep_sec_));
    }

    return false;
  }

  void close_camera()
  {
    if (cap_) {
      try {
        cap_->release();
      } catch (...) {
      }
    }

    cap_.reset();
    camera_ok_ = false;
  }

  // =========================================================
  // TIMER
  // =========================================================
  void timer_callback()
  {
    double now = now_sec();

    if (!cap_ || !cap_->isOpened()) {
      camera_ok_ = false;

      if (now - last_retry_time_ >= retry_period_sec_) {
        last_retry_time_ = now;
        RCLCPP_WARN(get_logger(), "\U0001F501 Camera pipeline not opened. Retrying...");
        start_pipeline();
      }

      publish_status(false, "pipeline_not_opened");
      return;
    }

    cv::Mat frame;
    bool ok = cap_->read(frame);

    if (!ok || frame.empty()) {
      failed_reads_++;
      camera_ok_ = false;
      last_error_ = "frame_read_failed";

      if (failed_reads_ == 1 || failed_reads_ % 30 == 0) {
        RCLCPP_WARN(
          get_logger(),
          "\u26A0\uFE0F Camera frame read failed | count=%d, pipeline=%s",
          failed_reads_, active_pipeline_name_.c_str());
      }

      if (failed_reads_ >= 60) {
        RCLCPP_WARN(get_logger(), "\U0001F501 Too many failed reads. Restarting pipeline...");
        start_pipeline();
      }

      publish_status(false, "frame_read_failed");
      return;
    }

    failed_reads_ = 0;
    camera_ok_ = true;
    last_error_ = "ok";

    if (flip_horizontal_) {
      cv::flip(frame, frame, 1);
    }

    if (flip_vertical_) {
      cv::flip(frame, frame, 0);
    }

    publish_image(frame);
    publish_status(true, "ok");
  }

  // =========================================================
  // PUBLISH
  // =========================================================
  void publish_image(const cv::Mat & frame)
  {
    try {
      std_msgs::msg::Header header;
      header.stamp = get_clock()->now();
      header.frame_id = frame_id_;

      auto msg = cv_bridge::CvImage(header, "bgr8", frame).toImageMsg();
      image_pub_->publish(*msg);
      frame_counter_++;
    } catch (const std::exception & e) {
      last_error_ = std::string("publish_image_failed: ") + e.what();
      RCLCPP_WARN(get_logger(), "\u26A0\uFE0F Failed to publish image: %s", e.what());
    }
  }

  void publish_status(bool ok, const std::string & reason)
  {
    if (!publish_status_enabled_) {
      return;
    }

    double now = now_sec();

    // Не спамимо статусом
    if (now - last_status_time_ < 1.0) {
      return;
    }

    last_status_time_ = now;

    std::ostringstream json;
    json << "{"
         << "\"ok\":" << (ok ? "true" : "false") << ","
         << "\"reason\":\"" << reason << "\","
         << "\"device\":\"" << device_ << "\","
         << "\"gst_source\":\"" << gst_source_ << "\","
         << "\"active_pipeline\":\"" << active_pipeline_name_ << "\","
         << "\"width\":" << width_ << ","
         << "\"height\":" << height_ << ","
         << "\"fps\":" << fps_ << ","
         << "\"image_topic\":\"" << image_topic_ << "\","
         << "\"frame_counter\":" << frame_counter_ << ","
         << "\"failed_reads\":" << failed_reads_ << ","
         << "\"last_error\":\"" << last_error_ << "\","
         << "\"stamp\":" << now
         << "}";

    auto msg = std_msgs::msg::String();
    msg.data = json.str();
    status_pub_->publish(msg);
  }

  // =========================================================
  // MEMBERS
  // =========================================================
  std::string device_;
  int width_{640};
  int height_{480};
  double fps_{15.0};
  int fps_int_{15};

  std::string image_topic_;
  std::string status_topic_;
  std::string frame_id_;

  std::string gst_source_;
  std::string custom_pipeline_;

  bool publish_status_enabled_{true};
  double retry_period_sec_{5.0};

  int warmup_read_attempts_{25};
  double warmup_sleep_sec_{0.05};

  bool flip_horizontal_{false};
  bool flip_vertical_{false};

  std::shared_ptr<cv::VideoCapture> cap_;

  bool camera_ok_{false};
  std::string active_pipeline_name_;
  std::string active_pipeline_text_;
  std::string last_error_;

  int frame_counter_{0};
  int failed_reads_{0};

  double last_retry_time_{0.0};
  double last_status_time_{0.0};

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<CameraCaptureNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
