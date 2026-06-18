/// Threadsave pool allocated circular buffer
/// \author Mattis Kieffer (kieffer@igmr.rwth-aachen.de)
///
/// \copyright Copyright 2025 Institut für Getriebetechnik, Maschinendynamik und
/// Robotik, RWTH-Aachen University. All rights reserved.
///
/// \license SPDX-License-Identifier: LGPL-3.0-or-later (see LICENSE at repo root)

#include "scancontrol_lls_driver/buffer.hpp"

namespace scancontrol_lls
{

FrameMessage::FrameMessage(
  std::pmr::memory_resource & pool,
  const rclcpp::Time & rxTime,
  size_t size)
: m_rxTime(rxTime), m_frame(size, FrameBuffer::allocator_type(&pool)) {}

std::span<unsigned char> FrameMessage::data() noexcept {return {m_frame};}

std::span<const unsigned char> FrameMessage::data() const noexcept
{
  return {m_frame};
}

const rclcpp::Time & FrameMessage::rxTime() const noexcept {return m_rxTime;}


Buffer::Buffer(size_t frame_size, size_t num_frames)
: m_capacity(num_frames),
  m_unread(0),
  m_buffer_resource(std::pmr::pool_options{
    .max_blocks_per_chunk = num_frames,
    .largest_required_pool_block = sizeof(FrameTypeT)}),
  m_frame_resource{
    std::pmr::pool_options{.max_blocks_per_chunk = num_frames,
      .largest_required_pool_block = frame_size}},
  m_container(
    std::pmr::polymorphic_allocator<FrameTypeT>(&m_buffer_resource)) {}

Buffer::~Buffer() noexcept {shutdown();}


Buffer::FrameTypeT Buffer::newFrame(size_t size, const rclcpp::Time & timestamp)
{
  return FrameTypeT(m_frame_resource, timestamp, size);
}

void Buffer::push_frame(FrameTypeT && frame)
{
  std::unique_lock lock(m_mutex);
  if (m_shutdown) {
    return;
  }
  ++m_pending_pushes;
  // The predicate must observe m_shutdown as well; otherwise a producer blocked
  // on a full buffer would never wake on shutdown and would deadlock shutdown()
  // (which waits for m_pending_pushes to drain).
  m_not_full.wait(lock, [&]() {return m_unread < m_capacity || m_shutdown;});
  if (!m_shutdown) {
    m_container.push(std::move(frame));
    ++m_unread;
    m_not_empty.notify_one();
  }
  --m_pending_pushes;
  if (m_pending_pushes == 0) {
    m_not_full.notify_all();  // allow shutdown() to proceed
  }
}


std::optional<Buffer::FrameTypeT> Buffer::pop_frame()
{
  if (m_shutdown) {
    return std::nullopt;
  }
  std::unique_lock lock(m_mutex);
  m_not_empty.wait(lock, [&]() {return m_unread > 0 || m_shutdown;});
  if (m_shutdown) {
    return std::nullopt;
  } else {
    std::optional<FrameTypeT> frame(std::move(m_container.front()));
    m_container.pop();
    --m_unread;
    lock.unlock();
    m_not_full.notify_one();
    return frame;
  }
}


void Buffer::shutdown()
{
  {
    std::unique_lock lock(m_mutex);
    m_shutdown = true;
  }
  m_not_empty.notify_all();
  m_not_full.notify_all();
  std::unique_lock lock(m_mutex);
  m_not_full.wait(lock, [&]() {return m_pending_pushes == 0;});
}

}  // namespace scancontrol_lls
