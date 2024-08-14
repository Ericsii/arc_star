#include <arc_star/arc_star_node.hpp>

#include <event_camera_codecs/evt3_encoder.h>
#include <event_camera_codecs/mono_encoder.h>
#include <functional>
#include <rclcpp_components/register_node_macro.hpp>

namespace arc_star {

struct Event {
  uint64_t ts;
  uint16_t ex;
  uint16_t ey;
  uint8_t polarity;
};

static std::vector<Event> corners;

ArcStarNode::ArcStarNode(const rclcpp::NodeOptions &options)
    : Node("arc_star_node", options) {
  using namespace std::placeholders;

  // Initialize the detector
  auto arc_star_param = acd::ArcStarDetector::Param();
  arc_star_param.kSensorWidth_ = this->declare_parameter("sensor_width", 640);
  arc_star_param.kSensorHeight_ = this->declare_parameter("sensor_height", 480);
  detector_ = std::make_shared<acd::ArcStarDetector>(arc_star_param);

  // Initialize the event packet decoder factory
  decoder_factory_ = std::make_shared<
      event_camera_codecs::DecoderFactory<EventPacket, ArcStarNode>>();

  // Create event encoder for publishing corner events
  encoder_type_ = this->declare_parameter("encoder_type", "evt3");
  encoder_ = event_camera_codecs::Encoder::newInstance(encoder_type_);

  event_sub_ = this->create_subscription<EventPacket>(
      "event", 10, std::bind(&ArcStarNode::event_callback, this, _1));
  event_pub_ = this->create_publisher<EventPacket>("corner", 10);
}

auto ArcStarNode::event_callback(
    const event_camera_codecs::EventPacketConstSharedPtr &msg) -> void {
  // Decode the event packet
  auto decoder = decoder_factory_->getInstance(*msg);
  if (!decoder) {
    RCLCPP_ERROR(this->get_logger(),
                 "Failed to create decoder for event packet");
    return;
  }

  corners.clear();
  corners.reserve(msg->events.size() / 14);
  decoder->decode(*msg, this);

  if (corners.empty() || event_pub_->get_subscription_count() == 0) {
    return;
  }

  // Encode the corner events
  EventPacket corner_msg;
  corner_msg.header = msg->header;
  corner_msg.encoding = encoder_type_;
  corner_msg.height = msg->height;
  corner_msg.width = msg->width;
  corner_msg.time_base = msg->time_base;
  corner_msg.seq = seq_++;
  corner_msg.events.reserve(corners.size());

  encoder_->setBuffer(&corner_msg.events);
  encoder_->setSensorTime(corners[0].ts);
  for (const auto &corner : corners) {
    encoder_->encodeCD(corner.ts - corners[0].ts, corner.ex, corner.ey,
                       corner.polarity);
  }

  event_pub_->publish(corner_msg);
}

auto ArcStarNode::eventCD(uint64_t ts, uint16_t ex, uint16_t ey,
                          uint8_t polarity) -> void {
  // Detect corners
  double time = ts / 1e9;
  if (!detector_) {
    RCLCPP_ERROR(this->get_logger(), "Detector is not initialized");
    return;
  }
  if (detector_->isCorner(time, ex, ey, static_cast<bool>(polarity))) {
    corners.push_back({ts, ex, ey, polarity});
  }
}

auto ArcStarNode::eventExtTrigger(uint64_t, uint8_t, uint8_t) -> void {}

auto ArcStarNode::finished() -> void {}

auto ArcStarNode::rawData(const char *, size_t) -> void {}

} // namespace arc_star

RCLCPP_COMPONENTS_REGISTER_NODE(arc_star::ArcStarNode)