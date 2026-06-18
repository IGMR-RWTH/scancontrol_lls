/// Threadsave pool allocated circular buffer
/// \author Mattis Kieffer (kieffer@igmr.rwth-aachen.de)
///
/// \copyright Copyright 2025 Institut für Getriebetechnik, Maschinendynamik und
/// Robotik, RWTH-Aachen University. All rights reserved.
///
/// \license SPDX-License-Identifier: LGPL-3.0-or-later (see LICENSE at repo root)
#ifndef SCANCONTROL_LLS_DRIVER__BUFFER_HPP_
#define SCANCONTROL_LLS_DRIVER__BUFFER_HPP_
#include <atomic>
#include <condition_variable>
#include <memory>
#include <memory_resource>
#include <mutex>
#include <optional>
#include <queue>
#include <span>  // NOLINT
#include <utility>
#include <vector>

#include "rclcpp/time.hpp"

namespace scancontrol_lls
{

/// @brief Class holding a received frame from the Sensor (raw data) and the
/// frames Timestamp.
class FrameMessage
{
public:
  using FrameBuffer = std::pmr::vector<unsigned char>;
  FrameMessage(const FrameMessage &) = delete;
  FrameMessage & operator=(const FrameMessage &) = delete;
  FrameMessage(FrameMessage &&) noexcept = default;
  FrameMessage & operator=(FrameMessage &&) noexcept = default;
  FrameMessage() = delete;

  /// @brief Construct a Frame on the provided allocation pool
  /// @param pool Memory pool on which to allocate the frame
  /// @param rxTime receive or acquisition time for the frame
  /// @param size size of the frame in bytes
  explicit FrameMessage(
    std::pmr::memory_resource & pool,
    const rclcpp::Time & rxTime = rclcpp::Time{},
    size_t size = 0);

  /// @brief return a view on the data of this frame
  /// @return span over the data of this frame as raw bytes
  std::span<unsigned char> data() noexcept;
  /// @brief return an immutable view on the data of this frame
  /// @return span over the data of this frame as raw bytes
  std::span<const unsigned char> data() const noexcept;
  /// @brief Rx or acquisition time of this frame
  /// @return the frames Timestamp
  const rclcpp::Time & rxTime() const noexcept;

private:
  rclcpp::Time m_rxTime{};
  FrameBuffer m_frame{};
};

/// @brief Threadsafe FIFO buffer with preallocated memory resources.
class Buffer
{
public:
  using FrameTypeT = FrameMessage;
  using QueueContainerT = std::pmr::deque<FrameTypeT>;

  /// @brief construct the Buffer and initialize its memory resources
  /// @param frame_size the maximum site in bytes that is used by FrameTypeT to
  /// allocate its data storage.
  /// @param num_frames the maximum number of frames
  Buffer(size_t frame_size, size_t num_frames);
  ~Buffer() noexcept;

  /// @brief Allocate a new frame with the given size and timestamp
  /// @param size the number of bytes to allocate for the frames storage
  /// @param timestamp the Timestamp of the frame
  /// @return the allocated Frame
  FrameTypeT newFrame(size_t size, const rclcpp::Time & timestamp);
  /// @brief push a Frame to the back of the FIFO Queue
  /// @param frame Frame to add to the queue
  void push_frame(FrameTypeT && frame);

  /// @brief Retrieve the frame on the Front of the Fifo queue.
  /// Blocks if queue is empty, and no shutdown was requested.
  /// @return an Optional holding the frame or std::nullopt if no frame could be
  /// retrieved and a shutdown was requested.
  std::optional<FrameTypeT> pop_frame();

  /// @brief Signal a shutdown to the Buffer.
  /// this unblocks all pending calls to push_frame and pop_frame.
  void shutdown();

private:
  const size_t m_capacity;
  std::atomic_size_t m_unread{0};
  std::atomic_size_t m_pending_pushes{0};
  std::mutex m_mutex{};
  std::condition_variable m_not_empty{};
  std::condition_variable m_not_full{};
  std::atomic_bool m_shutdown{false};
  std::pmr::unsynchronized_pool_resource m_buffer_resource;
  std::pmr::synchronized_pool_resource m_frame_resource;
  std::queue<FrameTypeT, QueueContainerT> m_container;
};

}  // namespace scancontrol_lls

#endif  // SCANCONTROL_LLS_DRIVER__BUFFER_HPP_
