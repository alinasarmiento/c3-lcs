#pragma once

#include <string>
#include <vector>

#include "systems/framework/timestamped_vector.h"

namespace dairlib {
namespace systems {

/// PushbotKinematicsVector stores the robot output as a TimestampedVector
///    * positions
///    * velocities
template <typename T>
class PushbotKinematicsVector : public TimestampedVector<T> {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PushbotKinematicsVector)

  PushbotKinematicsVector() = default;

  explicit PushbotKinematicsVector(int num_joint_positions,
                                  int num_joint_velocities)
      : TimestampedVector<T>(num_joint_positions +
                             num_joint_velocities),
        num_joint_positions_(num_joint_positions),
        num_joint_velocities_(num_joint_velocities),
        joint_positions_start_(0),
        joint_velocities_start_(num_joint_positions_),
        num_positions_(num_joint_positions_),
        num_velocities_(num_joint_velocities_) {
  }

  /// Constructs a OutputVector with the specified positions and velocities.
  explicit PushbotKinematicsVector(
      const drake::VectorX<T>& joint_positions,
      const drake::VectorX<T>& joint_velocities)
      : PushbotKinematicsVector(
            joint_positions.size(),
            joint_velocities.size()) {
    this->SetJointPositions(joint_positions);
    this->SetJointVelocities(joint_velocities);
  }

  void SetJointPositions(drake::VectorX<T> positions) {
    DRAKE_DEMAND(positions.size() == num_joint_positions_);
    this->get_mutable_data().segment(joint_positions_start_,
                                     num_joint_positions_) = positions;
  }

  void SetJointVelocities(drake::VectorX<T> velocities) {
    DRAKE_DEMAND(velocities.size() == num_joint_velocities_);
    this->get_mutable_data().segment(joint_velocities_start_,
                                     num_joint_velocities_) = velocities;
  }

  void SetState(drake::VectorX<T> state) {
    DRAKE_DEMAND(state.size() == this->data_size());
    this->get_mutable_data().segment(joint_positions_start_,
                                     this->data_size()) = state;
  }

  /// Returns a const state vector
  const drake::VectorX<T> GetState() const {
    return this->get_data().segment(joint_positions_start_,
                                    this->data_size());
  }

  /// Returns a const positions vector for the end effector
  const drake::VectorX<T> GetJointPositions() const {
    return this->get_data().segment(joint_positions_start_,
                                    num_joint_positions_);
  }

  /// Returns a const positions vector for the end effector
  const drake::VectorX<T> GetJointVelocities() const {
    return this->get_data().segment(joint_velocities_start_,
                                    num_joint_velocities_);
  }

  /// Returns a const velocities vector
  const drake::VectorX<T> GetVelocities() const {
    return this->get_data().segment(
        joint_velocities_start_,
        num_joint_velocities_);
  }

  /// Returns a const positions vector
  const drake::VectorX<T> GetPositions() const {
    return this->get_data().segment(
        joint_positions_start_,
        num_joint_positions_);
  }

  /// Returns a mutable positions vector
  Eigen::Map<drake::VectorX<T>> GetMutablePositions() {
    auto data = this->get_mutable_data().segment(
        joint_positions_start_,
        num_joint_positions_);
    return Eigen::Map<drake::VectorX<T>>(&data(0), data.size());
  }

  /// Returns a mutable velocities vector
  Eigen::Map<drake::VectorX<T>> GetMutableVelocities() {
    auto data = this->get_mutable_data().segment(
        joint_velocities_start_,
        num_joint_velocities_);
    return Eigen::Map<drake::VectorX<T>>(&data(0), data.size());
  }

  /// Returns a mutable state vector
  Eigen::Map<drake::VectorX<T>> GetMutableState() {
    auto data = this->get_mutable_data().segment(joint_positions_start_,
                                                 this->data_size());
    return Eigen::Map<drake::VectorX<T>>(&data(0), data.size());
  }

 protected:
  virtual PushbotKinematicsVector<T>* DoClone() const {
    return new PushbotKinematicsVector<T>(
        num_joint_positions_,
        num_joint_velocities_);
  }

 private:
  const int num_joint_positions_;
  const int num_joint_velocities_;
  const int joint_positions_start_;
  const int joint_velocities_start_;

  const int num_positions_;
  const int num_velocities_;
};

}  // namespace systems
}  // namespace dairlib
