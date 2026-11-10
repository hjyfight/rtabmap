#include <map>
#include <memory>
#include <optional>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <ctime>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.h>
#include <rtabmap_conversions/MsgConversion.h>
#include <rtabmap_msgs/msg/map_data.hpp>
#include <rtabmap_msgs/msg/map_graph.hpp>
#include <rtabmap_msgs/srv/get_map.hpp>
#include <rtabmap_semantic_msgs/msg/keyframe.hpp>

class KeyframeBridge : public rclcpp::Node
{
public:
  KeyframeBridge() : Node("semantic_keyframe_bridge")
  {
    const auto keyframeTopic = declare_parameter("keyframe_topic", "/semantic/keyframes");
    const auto mapTopic = declare_parameter("map_data_topic", "/mapData");
    const auto service = declare_parameter("map_service", "/rtabmap/get_map_data");
    replay_ = declare_parameter("replay_existing", true);
    maxPending_ = declare_parameter("max_pending_frames", 32);
    frames_ = create_publisher<rtabmap_semantic_msgs::msg::Keyframe>(keyframeTopic, 5);
    timing_ = create_publisher<std_msgs::msg::String>("/semantic/bridge_timing", 100);
    std::ifstream boot("/proc/sys/kernel/random/boot_id");
    std::getline(boot, clockId_);
    runId_ = std::to_string(monotonicNs());
    graph_ = create_publisher<rtabmap_msgs::msg::MapGraph>(
      "/semantic/graph", rclcpp::QoS(1).reliable().transient_local());
    sub_ = create_subscription<rtabmap_msgs::msg::MapData>(mapTopic, 5,
      [this](rtabmap_msgs::msg::MapData::ConstSharedPtr msg) { process(*msg, false); });
    ack_ = create_subscription<std_msgs::msg::Int32>("/semantic/processed_node", 10,
      [this](std_msgs::msg::Int32::ConstSharedPtr msg) {
        if (inflight_ && inflight_->node_id == msg->data) {
          seen_[inflight_->node_id] = inflight_->node_stamp;
          inflight_.reset();
        }
      });
    dispatchTimer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() { dispatch(); });
    client_ = create_client<rtabmap_msgs::srv::GetMap>(service);
    // Request saved nodes on startup, allowing restart and offline replay.
    timer_ = create_wall_timer(std::chrono::seconds(2), [this]() {
      if (!replay_ || !client_->service_is_ready() || frames_->get_subscription_count() == 0)
        return;
      auto request = std::make_shared<rtabmap_msgs::srv::GetMap::Request>();
      request->global_map = true;
      request->optimized = true;
      request->graph_only = false;
      client_->async_send_request(request,
        [this](rclcpp::Client<rtabmap_msgs::srv::GetMap>::SharedFuture response) {
          process(response.get()->data, true);
          RCLCPP_INFO(get_logger(), "Saved keyframes requested; semantic mapper deduplicates saved observations.");
        });
      timer_->cancel();
    });
  }

private:
  struct Pending {
    rtabmap_msgs::msg::Node node;
    int64_t received;
    bool replay;
  };

  static int64_t monotonicNs()
  {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return int64_t(ts.tv_sec)*1000000000LL + ts.tv_nsec;
  }

  void trace(const Pending & item, const char * event, int64_t dispatch = 0,
             int64_t decoded = 0, int64_t published = 0)
  {
    std::ostringstream out;
    out << std::setprecision(17)
        << "{\"schema\":1,\"source\":\"keyframe_bridge\",\"run_id\":\"" << runId_
        << "\",\"clock_id\":\"" << clockId_ << "\",\"sequence\":" << ++sequence_
        << ",\"event\":\"" << event << "\",\"event_ns\":" << monotonicNs()
        << ",\"node_id\":" << item.node.id << ",\"node_stamp\":" << item.node.stamp
        << ",\"session_id\":" << item.node.map_id << ",\"received_ns\":" << item.received
        << ",\"replay\":" << (item.replay ? "true" : "false")
        << ",\"dispatch_ns\":" << dispatch << ",\"decoded_ns\":" << decoded
        << ",\"published_ns\":" << published << ",\"pending_depth\":" << pending_.size() << "}";
    std_msgs::msg::String msg;
    msg.data = out.str();
    timing_->publish(msg);
  }

  void process(const rtabmap_msgs::msg::MapData & data, bool replay)
  {
    const auto received = monotonicNs();
    for (const auto & node : data.nodes) {
      const auto old = knownStamps_.find(node.id);
      if (old != knownStamps_.end() && old->second != node.stamp) {
        for (const auto & item : pending_) trace(item.second, "dropped_reset");
        pending_.clear();
        seen_.clear();
        knownStamps_.clear();
        inflight_.reset();
        replaying_ = false;
        RCLCPP_WARN(get_logger(), "Graph node IDs were reused; discarded pending frames from the old map.");
        break;
      }
    }
    auto graph = data.graph;
    graph.header = data.header;
    graph_->publish(graph);
    for (const auto & node : data.nodes)
    {
      if (node.id <= 0 || (seen_.count(node.id) && seen_[node.id] == node.stamp)) continue;
      knownStamps_[node.id] = node.stamp;
      if (inflight_ && inflight_->node_id == node.id && inflight_->node_stamp == node.stamp) continue;
      // Keep only RGB-D payloads, not LiDAR scans/features, in the pending queue.
      rtabmap_msgs::msg::Node copy;
      copy.id = node.id; copy.stamp = node.stamp; copy.map_id = node.map_id;
      copy.data.header = node.data.header;
      copy.data.left = node.data.left; copy.data.right = node.data.right;
      copy.data.left_compressed = node.data.left_compressed;
      copy.data.right_compressed = node.data.right_compressed;
      copy.data.left_camera_info = node.data.left_camera_info;
      copy.data.right_camera_info = node.data.right_camera_info;
      copy.data.local_transform = node.data.local_transform;
      if (pending_.count(node.id)) {
        pending_.at(node.id).node = std::move(copy);
        continue; // Refresh data, but preserve the original queue arrival time.
      }
      pending_[node.id] = Pending{std::move(copy), received, replay};
      trace(pending_.at(node.id), "queued");
      // Offline replay is explicitly requested and is delivered completely with backpressure.
      if (!replay && !replaying_ && maxPending_ > 0 && pending_.size() > static_cast<size_t>(maxPending_)) {
        trace(pending_.begin()->second, "dropped_overflow");
        pending_.erase(pending_.begin());
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "Inference is behind: skipped oldest pending keyframe (SLAM is unaffected).");
      }
    }
    if (replay) replaying_ = true;
  }

  void dispatch()
  {
    if (frames_->get_subscription_count() == 0) return;
    if (inflight_) {
      if (std::chrono::steady_clock::now() - sent_ > std::chrono::seconds(10)) {
        frames_->publish(*inflight_);
        sent_ = std::chrono::steady_clock::now();
      }
      return;
    }
    if (pending_.empty()) { replaying_ = false; return; }
    auto item = std::move(pending_.begin()->second);
    auto & node = item.node;
    pending_.erase(pending_.begin());
    const auto dispatch = monotonicNs();
      try
      {
        auto sensor = rtabmap_conversions::sensorDataFromROS(node.data);
        cv::Mat rgb, depth;
        sensor.uncompressData(&rgb, &depth);
        if (rgb.empty() || depth.empty() || sensor.cameraModels().size() != 1) {
          trace(item, "invalid_input"); return;
        }
        const auto & camera = sensor.cameraModels().front();
        if (!camera.isValidForProjection() || camera.localTransform().isNull()) {
          trace(item, "invalid_input"); return;
        }
        if (rgb.type() == CV_8UC1) cv::cvtColor(rgb, rgb, cv::COLOR_GRAY2BGR);
        if (rgb.type() != CV_8UC3 || (depth.type() != CV_16UC1 && depth.type() != CV_32FC1)) {
          trace(item, "invalid_input"); return;
        }
        rtabmap_semantic_msgs::msg::Keyframe frame;
        frame.header = node.data.header;
        frame.node_id = node.id;
        frame.session_id = node.map_id;
        frame.node_stamp = node.stamp;
        frame.rgb = *cv_bridge::CvImage(frame.header, "bgr8", rgb).toImageMsg();
        frame.depth = *cv_bridge::CvImage(frame.header,
          depth.type() == CV_16UC1 ? "16UC1" : "32FC1", depth).toImageMsg();
        rtabmap_conversions::cameraModelToROS(camera, frame.camera_info);
        rtabmap_conversions::transformToGeometryMsg(camera.localTransform(), frame.camera_to_node);
        const auto decoded = monotonicNs();
        frames_->publish(frame);
        trace(item, "dispatched", dispatch, decoded, monotonicNs());
        inflight_ = std::move(frame);
        sent_ = std::chrono::steady_clock::now();
      }
      catch (const std::exception & error)
      {
        trace(item, "decode_failed");
        RCLCPP_WARN(get_logger(), "Cannot decode node %d: %s", node.id, error.what());
      }
  }
  bool replay_;
  bool replaying_ = false;
  int maxPending_;
  std::map<int, double> seen_;
  std::map<int, double> knownStamps_;
  std::map<int, Pending> pending_;
  std::optional<rtabmap_semantic_msgs::msg::Keyframe> inflight_;
  std::chrono::steady_clock::time_point sent_;
  std::string clockId_, runId_;
  uint64_t sequence_ = 0;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr timing_;
  rclcpp::Publisher<rtabmap_semantic_msgs::msg::Keyframe>::SharedPtr frames_;
  rclcpp::Publisher<rtabmap_msgs::msg::MapGraph>::SharedPtr graph_;
  rclcpp::Subscription<rtabmap_msgs::msg::MapData>::SharedPtr sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr ack_;
  rclcpp::Client<rtabmap_msgs::srv::GetMap>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr dispatchTimer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KeyframeBridge>());
  rclcpp::shutdown();
  return 0;
}
