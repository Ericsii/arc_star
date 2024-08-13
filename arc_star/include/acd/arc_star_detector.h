#ifndef ARC_STAR_DETECTOR_H
#define ARC_STAR_DETECTOR_H

#include <Eigen/Dense>

namespace acd { // Asynchronous Corner Detector

class ArcStarDetector {
public:
  struct Param {
    int kSensorWidth_ = 640;
    int kSensorHeight_ = 480;
    double filter_threshold_ = 0.5;
    int kSmallCircleSize = 16;
    int kLargeCircleSize = 20;
    int kSmallMinThresh = 3;
    int kSmallMaxThresh = 6;
    int kLargeMinThresh = 4;
    int kLargeMaxThresh = 8;
    int kBorderLimit = 4;

    Param() {}
  };

  explicit ArcStarDetector(Param param = ArcStarDetector::Param());
  ~ArcStarDetector();

  bool isCorner(double et, int ex, int ey, bool ep);

private:
  // Circular Breshenham Masks
  const int kSmallCircle_[16][2];
  const int kLargeCircle_[20][2];

  // Parameters
  Param param_;

  // Surface of Active Events
  Eigen::MatrixXd sae_[2];
  Eigen::MatrixXd sae_latest_[2];
};

} // namespace acd

#endif // ARC_STAR_DETECTOR_H
