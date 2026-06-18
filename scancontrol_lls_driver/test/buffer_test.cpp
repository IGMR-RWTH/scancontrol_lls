/// Test for the fifo buffer implementation
/// \author Mattis Kieffer (kieffer@igmr.rwth-aachen.de)
///
/// \copyright Copyright 2025 Institut für Getriebetechnik, Maschinendynamik und
/// Robotik, RWTH-Aachen University. All rights reserved.
///
/// \license SPDX-License-Identifier: LGPL-3.0-or-later (see LICENSE at repo root)
#include "scancontrol_lls_driver/buffer.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <future>
#include <latch>  // NOLINT
#include <optional>
#include <thread>
#include <vector>

#include "rclcpp/time.hpp"

using scancontrol_lls::Buffer;
using scancontrol_lls::FrameMessage;

class EventLog
{
public:
  void log(const std::string & enrtry)
  {
    std::unique_lock lck(m_mtx);
    m_log.push_back(enrtry);
  }
  std::vector<std::string> getLog()
  {
    std::unique_lock lck(m_mtx);
    return m_log;
  }

private:
  std::vector<std::string> m_log;
  std::mutex m_mtx;
};

// Helpers to fill and verify frame content
void fill_pattern(FrameMessage & f, unsigned char start_val)
{
  auto d = f.data();
  for (size_t i = 0; i < d.size(); ++i) {
    d[i] = static_cast<unsigned char>(start_val + static_cast<unsigned char>(i));
  }
}

void expect_pattern(const FrameMessage & f, unsigned char start_val)
{
  auto d = f.data();
  for (size_t i = 0; i < d.size(); ++i) {
    ASSERT_EQ(d[i], static_cast<unsigned char>(start_val + static_cast<unsigned char>(i)))
      << "Mismatch at index " << i;
  }
}

// Basic construction / newFrame properties
TEST(BufferTest, NewFrameHasCorrectSizeAndTimestamp)
{
  Buffer buf(/*frame_size*/ 256, /*num_frames*/ 4);

  rclcpp::Time ts(1, 2);  // 1 second + 2 nanoseconds
  auto f = buf.newFrame(10, ts);

  EXPECT_EQ(f.data().size(), 10U);
  EXPECT_EQ(f.rxTime().nanoseconds(), ts.nanoseconds());
}

// Single-threaded push and pop, including data integrity and timestamp
TEST(BufferTest, PushPopSingleThread)
{
  Buffer buf(64, 2);

  rclcpp::Time ts(3, 4);
  auto f = buf.newFrame(5, ts);
  fill_pattern(f, 10);

  buf.push_frame(std::move(f));
  auto got = buf.pop_frame().value();

  ASSERT_EQ(got.data().size(), 5U);
  expect_pattern(got, 10);
  EXPECT_EQ(got.rxTime().nanoseconds(), ts.nanoseconds());
}

// FIFO order across multiple frames
TEST(BufferTest, FifoOrderPreserved)
{
  Buffer buf(64, 3);

  rclcpp::Time ts1(0, 1);
  rclcpp::Time ts2(0, 2);
  rclcpp::Time ts3(0, 3);

  auto f1 = buf.newFrame(3, ts1);
  auto f2 = buf.newFrame(4, ts2);
  auto f3 = buf.newFrame(5, ts3);

  fill_pattern(f1, 11);
  fill_pattern(f2, 22);
  fill_pattern(f3, 33);

  buf.push_frame(std::move(f1));
  buf.push_frame(std::move(f2));
  buf.push_frame(std::move(f3));

  auto g1 = buf.pop_frame().value();
  auto g2 = buf.pop_frame().value();
  auto g3 = buf.pop_frame().value();

  ASSERT_EQ(g1.data().size(), 3U);
  ASSERT_EQ(g2.data().size(), 4U);
  ASSERT_EQ(g3.data().size(), 5U);

  expect_pattern(g1, 11);
  expect_pattern(g2, 22);
  expect_pattern(g3, 33);

  EXPECT_EQ(g1.rxTime().nanoseconds(), ts1.nanoseconds());
  EXPECT_EQ(g2.rxTime().nanoseconds(), ts2.nanoseconds());
  EXPECT_EQ(g3.rxTime().nanoseconds(), ts3.nanoseconds());
}

// push_frame blocks when the buffer is full until a pop occurs
TEST(BufferTest, PushBlocksWhenFullUntilPop)
{
  Buffer buf(16, 1);  // capacity 1
  EventLog log;
  rclcpp::Time ts1(0, 10);
  rclcpp::Time ts2(0, 20);

  auto f1 = buf.newFrame(1, ts1);
  f1.data()[0] = 42;
  buf.push_frame(std::move(f1));  // buffer now full

  std::latch start(1);
  std::latch finished(1);
  std::thread producer([&] {
      log.log("start T1");
      auto f2 = buf.newFrame(1, ts2);
      f2.data()[0] = 99;
      start.count_down();
      buf.push_frame(std::move(f2));  // should block until main thread pops
      finished.count_down();
      log.log("end T1");
    });
  start.wait();
  EXPECT_FALSE(finished.try_wait()) << "Producer finished too early, push didn't block";
  // Pop one to make room, which should unblock producer
  std::optional<FrameMessage> got1{};
  std::thread consumer([&] {
      log.log("start T2");
      got1.emplace(buf.pop_frame().value());
      log.log("end T2");
    });
  consumer.join();
  producer.join();
  ASSERT_TRUE(got1.has_value());
  ASSERT_EQ(got1->data().size(), 1U);
  EXPECT_EQ(got1->data()[0], 42);
  EXPECT_EQ(got1->rxTime().nanoseconds(), ts1.nanoseconds());

  EXPECT_TRUE(finished.try_wait());
  auto logs = log.getLog();
  EXPECT_STREQ(logs[0].c_str(), "start T1");
  EXPECT_STREQ(logs[1].c_str(), "start T2");
  EXPECT_STREQ(logs[2].c_str(), "end T2");
  EXPECT_STREQ(logs[3].c_str(), "end T1");
}

// pop_frame blocks when the buffer is empty until a push occurs
TEST(BufferTest, PopBlocksWhenEmptyUntilPush)
{
  Buffer buf(16, 2);

  std::promise<FrameMessage> prom;
  auto fut = prom.get_future();

  std::thread consumer([&] {prom.set_value(buf.pop_frame().value());});

  // Ensure pop blocks initially
  auto status_before = fut.wait_for(std::chrono::milliseconds(50));
  EXPECT_EQ(status_before, std::future_status::timeout) << "pop_frame returned too early";

  // Push frame to unblock
  rclcpp::Time ts(0, 123);
  auto f = buf.newFrame(2, ts);
  f.data()[0] = 7;
  f.data()[1] = 8;
  buf.push_frame(std::move(f));

  // Now it should complete
  auto status_after = fut.wait_for(std::chrono::milliseconds(500));
  EXPECT_EQ(status_after, std::future_status::ready) << "pop_frame did not return after push";

  auto got = fut.get();
  consumer.join();

  ASSERT_EQ(got.data().size(), 2U);
  EXPECT_EQ(got.data()[0], 7);
  EXPECT_EQ(got.data()[1], 8);
  EXPECT_EQ(got.rxTime().nanoseconds(), ts.nanoseconds());
}

// pop_frame unblocks on purge and returns an empty/default frame when no frames are available
TEST(BufferTest, ShutdownFreesBlockingThreads)
{
  Buffer buf(16, 2);
  std::latch started(1);
  std::latch stopped(1);
  std::promise<std::optional<FrameMessage>> prom;
  auto fut = prom.get_future();
  std::thread consumer([&] {
      started.count_down();
      prom.set_value(buf.pop_frame());  // should block until purge
      stopped.count_down();
    });
  started.wait();
  // Confirm it's blocking
  EXPECT_FALSE(stopped.try_wait());
  buf.shutdown();
  // if this fails, we wont be able to join.
  consumer.join();
  auto got = fut.get();
  // Expect default/empty frame because queue was empty at the time
  EXPECT_FALSE(got.has_value());
}

// shutdown() must unblock a producer that is waiting on a full buffer, and must
// itself return rather than deadlock waiting for in-flight pushes to drain.
TEST(BufferTest, ShutdownUnblocksFullBufferProducer)
{
  Buffer buf(16, 1);  // capacity 1

  rclcpp::Time ts1(0, 10);
  rclcpp::Time ts2(0, 20);

  auto f1 = buf.newFrame(1, ts1);
  f1.data()[0] = 1;
  buf.push_frame(std::move(f1));  // buffer now full

  std::latch started(1);
  std::promise<void> finished;
  auto fut = finished.get_future();
  std::thread producer([&] {
      auto f2 = buf.newFrame(1, ts2);
      f2.data()[0] = 2;
      started.count_down();
      buf.push_frame(std::move(f2));  // blocks: buffer is full
      finished.set_value();
    });

  started.wait();
  // Confirm the producer is actually blocked before we shut down.
  EXPECT_EQ(fut.wait_for(std::chrono::milliseconds(50)), std::future_status::timeout)
    << "producer should be blocked on a full buffer";

  buf.shutdown();  // must not deadlock and must release the blocked producer

  EXPECT_EQ(fut.wait_for(std::chrono::milliseconds(500)), std::future_status::ready)
    << "shutdown did not release the blocked producer";
  producer.join();
}

// Basic multi-producer / single-consumer stress test for concurrency and data integrity
TEST(BufferTest, MultiProducerSingleConsumerIntegrity)
{
  constexpr size_t capacity = 8;
  constexpr int producers = 3;
  constexpr int frames_per_producer = 25;
  Buffer buf(32, capacity);

  std::atomic<int> produced_count{0};
  std::atomic<int> consumed_count{0};

  // Each frame will be size 2: [producer_id, seq_index]
  auto producer_fn = [&](int id) {
      for (int i = 0; i < frames_per_producer; ++i) {
        auto f = buf.newFrame(2, rclcpp::Time(id, static_cast<uint32_t>(i)));
        auto d = f.data();
        d[0] = static_cast<unsigned char>(id);
        d[1] = static_cast<unsigned char>(i);
        buf.push_frame(std::move(f));
        produced_count.fetch_add(1, std::memory_order_relaxed);
      }
    };

  std::vector<std::thread> prod_threads;
  for (int p = 0; p < producers; ++p) {
    prod_threads.emplace_back(producer_fn, p);
  }

  // Consumer collects expected number of frames
  std::vector<std::vector<bool>> seen(producers, std::vector<bool>(frames_per_producer, false));
  std::thread consumer([&] {
      const int total = producers * frames_per_producer;
      while (consumed_count.load(std::memory_order_relaxed) < total) {
        auto frame = buf.pop_frame().value();
        auto d = frame.data();
        if (d.size() == 2) {
          int pid = d[0];
          int idx = d[1];
          if (pid >= 0 && pid < producers && idx >= 0 && idx < frames_per_producer) {
            seen[pid][idx] = true;
            consumed_count.fetch_add(1, std::memory_order_relaxed);
          } else {
            // Unexpected, fail early
            ADD_FAILURE() << "Consumer received out-of-range id/idx: " << pid << "/" << idx;
            break;
          }
        } else {
          // Should not occur in this test (no purge used)
          ADD_FAILURE() << "Consumer received empty or wrong-sized frame";
          break;
        }
      }
    });

  for (auto & t : prod_threads) {
    t.join();
  }
  consumer.join();

  EXPECT_EQ(produced_count.load(), producers * frames_per_producer);
  EXPECT_EQ(consumed_count.load(), producers * frames_per_producer);

  // Verify each expected frame was seen exactly once
  for (int p = 0; p < producers; ++p) {
    for (int i = 0; i < frames_per_producer; ++i) {
      EXPECT_TRUE(seen[p][i]) << "Missing frame for producer " << p << ", index " << i;
    }
  }
}
