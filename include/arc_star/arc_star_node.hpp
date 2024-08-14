#pragma once

#include <memory>

#include <event_camera_codecs/decoder.h>
#include <event_camera_codecs/decoder_factory.h>
#include <event_camera_codecs/encoder.h>
#include <rclcpp/rclcpp.hpp>

#include <acd/arc_star_detector.h>

namespace arc_star {

class ArcStarNode : public rclcpp::Node, event_camera_codecs::EventProcessor {
public:
  using EventPacket = event_camera_codecs::EventPacket;

  explicit ArcStarNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

  inline auto eventCD(uint64_t, uint16_t, uint16_t, uint8_t) -> void override;
  auto eventExtTrigger(uint64_t, uint8_t, uint8_t) -> void override;
  auto finished() -> void override;
  auto rawData(const char *, size_t) -> void override;

private:
  auto event_callback(const event_camera_codecs::EventPacketConstSharedPtr &msg)
      -> void;

  rclcpp::Subscription<EventPacket>::SharedPtr event_sub_;
  rclcpp::Publisher<EventPacket>::SharedPtr event_pub_;

  std::shared_ptr<acd::ArcStarDetector> detector_;
  std::shared_ptr<event_camera_codecs::DecoderFactory<EventPacket, ArcStarNode>>
      decoder_factory_;

  std::shared_ptr<event_camera_codecs::Encoder> encoder_;

  std::string encoder_type_ = "evt3";
  uint64_t seq_ = 0;
}; // class ArcStarNode

} // namespace arc_star