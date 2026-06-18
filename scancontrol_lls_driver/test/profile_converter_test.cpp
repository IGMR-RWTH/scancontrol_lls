/// Profile converter test file.
/// \author Mattis Kieffer (kieffer@igmr.rwth-aachen.de)
///
/// \copyright Copyright 2025 Institut für Getriebetechnik, Maschinendynamik und
/// Robotik, RWTH-Aachen University. All rights reserved.
///
/// \license SPDX-License-Identifier: LGPL-3.0-or-later (see LICENSE at repo root)
#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "scancontrol_lls_driver/profile_publisher.hpp"
#include "scancontrol_lls_driver/message_point.hpp"
#include "scancontrol_lls_driver/scanner_interface.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

using ::testing::_;
using ::testing::Invoke;
using ::testing::Mock;
using ::testing::Return;

namespace scancontrol_lls
{
// Test fixture for MessagePoint tests
class MessagePointTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Any setup code if needed
  }

  void TearDown() override
  {
    // Any cleanup code if needed
  }
};

// Test for GetFieldDef function
TEST_F(MessagePointTest, GetFieldDef_ReturnsCorrectFields)
{
  auto fields = MessagePoint::GetFieldDef();

  ASSERT_EQ(fields.size(), 8);

  EXPECT_EQ(fields[0].count, 1);
  EXPECT_EQ(fields[0].datatype, sensor_msgs::msg::PointField::FLOAT32);
  EXPECT_EQ(fields[0].name, "x");
  EXPECT_EQ(fields[0].offset, 0);
  EXPECT_EQ(fields[1].count, 1);
  EXPECT_EQ(fields[1].datatype, sensor_msgs::msg::PointField::FLOAT32);
  EXPECT_EQ(fields[1].name, "y");
  EXPECT_EQ(fields[1].offset, 4);
  EXPECT_EQ(fields[2].count, 1);
  EXPECT_EQ(fields[2].datatype, sensor_msgs::msg::PointField::FLOAT32);
  EXPECT_EQ(fields[2].name, "z");
  EXPECT_EQ(fields[2].offset, 8);
}

// Test for NewPointCloud function
TEST_F(MessagePointTest, NewPointCloud_CreatesValidPointCloud)
{
  size_t size = 10;

  auto point_cloud = MessagePoint::NewPointCloud(size);

  EXPECT_EQ(point_cloud.height, 1);
  EXPECT_EQ(point_cloud.width, size);

  // Check that data is resized correctly
  EXPECT_EQ(point_cloud.data.size(), sizeof(MessagePoint) * size);

  // Check point step and row step sizes
  EXPECT_EQ(point_cloud.point_step, sizeof(MessagePoint));
  EXPECT_EQ(point_cloud.row_step, size * point_cloud.point_step);

  // Ensure is_dense is true
  EXPECT_TRUE(point_cloud.is_dense);

  // Validate field definitions match expected values again if necessary...
}

// Test for NewInPlace function with valid index
TEST_F(MessagePointTest, NewInPlace_AllocatesMessageInBuffer)
{
  sensor_msgs::msg::PointCloud2 pc = MessagePoint::NewPointCloud(5);

  size_t idx = 2;
  MessagePoint * msg_point = MessagePoint::NewInPlace(pc, idx);

  ASSERT_NE(msg_point, nullptr);
  ASSERT_NO_THROW({msg_point->x = 1.0;});
}

// Test for NewInPlace function with invalid index (out of bounds)
TEST_F(MessagePointTest, NewInPlace_ThrowsError_WhenIndexOutOfBounds)
{
  sensor_msgs::msg::PointCloud2 pc = MessagePoint::NewPointCloud(5);

  size_t idx = 10;

  // Expecting an exception to be thrown when trying to allocate out of bounds.
  ASSERT_THROW({MessagePoint::NewInPlace(pc, idx);}, ScannerError);
}

}  // namespace scancontrol_lls

class MockPublisher
{
public:
  MOCK_METHOD(void, publish, (std::unique_ptr<sensor_msgs::msg::PointCloud2>));
};

class ProfileConverterTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Use a valid scanner type + resolution so ConvertProfile2Values succeeds.
    // 27xx accepts resolutions in [80, 640] in steps of 80; the raw frame must be
    // resolution * BYTES_PER_POINT bytes. A zeroed buffer is a well-formed (if
    // empty) profile that converts to X/Z values without error.
    m_resolution = 80;
    m_mockPublisher = std::make_shared<MockPublisher>();
    m_profileConverter = std::make_unique<scancontrol_lls::ProfilePublisher>(
      m_resolution, scanCONTROL27xx_25,
      [pub = m_mockPublisher](std::unique_ptr<sensor_msgs::msg::PointCloud2> pc) {
        pub->publish(std::move(pc));
      }, "test", 256);
  }

  // Size in bytes of a well-formed raw profile frame for m_resolution points.
  size_t frameBytes() const {return m_resolution * scancontrol_lls::constants::BYTES_PER_POINT;}

  void TearDown() override
  {
    // Clean up resources if needed.
    m_profileConverter.reset();
  }

  size_t m_resolution;
  std::shared_ptr<MockPublisher> m_mockPublisher;
  std::unique_ptr<scancontrol_lls::ProfilePublisher> m_profileConverter;
};

// Test emplaceData method
TEST_F(ProfileConverterTest, EmplaceData_AddsDataToQueue)
{
  std::vector<unsigned char> testData(frameBytes());  // well-formed (zeroed) profile

  EXPECT_CALL(*m_mockPublisher, publish(_)).Times(1);  // Expecting publish to be called once

  // Add data to the converter
  m_profileConverter->emplaceData(testData.data(), testData.size(), rclcpp::Time{});

  // Allow some time for processing in another thread
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

// Test that run processes data correctly
TEST_F(ProfileConverterTest, Run_ProcessesAndPublishesData)
{
  std::vector<unsigned char> testData(frameBytes());  // well-formed (zeroed) profile
  const auto expected_width = m_resolution;

  EXPECT_CALL(*m_mockPublisher, publish(_))
  .Times(1)
  .WillOnce(
    Invoke(
      [expected_width](std::unique_ptr<sensor_msgs::msg::PointCloud2> msg) {
        ASSERT_NE(msg.get(), nullptr);  // Ensure message is not null
        EXPECT_EQ(msg->width, expected_width);  // Check resolution matches expected width
      }));

  // Add data and trigger processing
  m_profileConverter->emplaceData(testData.data(), testData.size(), rclcpp::Time{});

  // Allow some time for processing in another thread
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
}
int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
