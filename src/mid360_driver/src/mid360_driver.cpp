/**
 * This file is part of Mid-360 driver.
 * Copyright (C) 2025  Yingjie Huang
 * Licensed under the MIT License. See License.txt in the project root for license information.
 */

#define _USE_MATH_DEFINES
#include "mid360_driver.h"
#include <chrono>
#include <cmath>

namespace mid360_driver {

    enum DataType : std::uint8_t {
        kLivoxLidarImuData = 0,
        kLivoxLidarCartesianCoordinateHighData = 0x01,
        kLivoxLidarCartesianCoordinateLowData = 0x02,
        kLivoxLidarSphericalCoordinateData = 0x03
    };

    enum TimestampType : std::uint8_t {
        kTimestampTypeNoSync = 0,   // 没有同步信号
        kTimestampTypeGptpOrPtp = 1,// gPTP 或 PTP 同步模式
        kTimestampTypeGps = 2       // GPS 同步模式
    };
    constexpr uint16_t POINTCLOUD_SEND_PORT = 56300;
    constexpr uint16_t POINTCLOUD_RECV_PORT = 56301;
    constexpr uint16_t IMU_SEND_PORT = 56400;
    constexpr uint16_t IMU_RECV_PORT = 56401;

    constexpr auto RESTART_DELAY = std::chrono::milliseconds(100);

#pragma pack(1)

    struct DataHeader {
        uint8_t version;        // 包协议版本：当前为0
        uint16_t length;        // 整个UDP数据段长度
        uint16_t time_interval; // 帧内点云采样时间(单位0.1us)；这帧点云数据中最后一个点减去第一个点时间
        uint16_t dot_num;       // 当前UDP包的点数目
        uint16_t udp_cnt;       // 点云UDP包计数，每个UDP包依次加1，点云帧起始包清0
        uint8_t frame_cnt;      // 点云帧计数，每帧点云(10Hz/15Hz等)依次加1
        DataType data_type;     // 数据类型
        TimestampType time_type;// 时间戳类型
        uint8_t reserved[12];   // 保留
        uint32_t crc32;         // timestamp+点云数据校验码，使用CRC-32算法
        uint64_t timestamp;     // 点云时间戳，单位: ns
    };

    struct Imu {
        float angular_velocity_x;
        float angular_velocity_y;
        float angular_velocity_z;
        float linear_acceleration_x;
        float linear_acceleration_y;
        float linear_acceleration_z;
    };

    struct CartesianHighPoint {
        int32_t x;           // unit:mm
        int32_t y;           // unit:mm
        int32_t z;           // unit:mm
        uint8_t reflectivity;// 反射率
        uint8_t tag;         // 标签
    };

    struct CartesianLowPoint {
        int16_t x;           // unit:mm
        int16_t y;           // unit:mm
        int16_t z;           // unit:mm
        uint8_t reflectivity;// 反射率
        uint8_t tag;         // 标签
    };

    struct SphericalPoint {
        uint32_t depth;      // 深度
        uint16_t theta;      // 天顶角[0, 18000], Unit: 0.01 度
        uint16_t phi;        // 方位角[0, 36000], Unit: 0.01 度
        uint8_t reflectivity;// 反射率
        uint8_t tag;         // 标签
    };

#pragma pack()

    void combine_4_bytes(std::size_t &seed, const unsigned char *bytes) {
        const std::size_t bytes_hash =
                (static_cast<std::size_t>(bytes[0]) << 24) |
                (static_cast<std::size_t>(bytes[1]) << 16) |
                (static_cast<std::size_t>(bytes[2]) << 8) |
                (static_cast<std::size_t>(bytes[3]));
        seed ^= bytes_hash + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }

    std::size_t IpAddressHasher::operator()(const asio::ip::address &addr) const noexcept {
        if (addr.is_v4()) {
            return std::hash<unsigned int>()(addr.to_v4().to_uint());
        } else {
            const asio::ip::address_v6::bytes_type bytes = addr.to_v6().to_bytes();
            std::size_t result = static_cast<std::size_t>(addr.to_v6().scope_id());
            combine_4_bytes(result, &bytes[0]);
            combine_4_bytes(result, &bytes[4]);
            combine_4_bytes(result, &bytes[8]);
            combine_4_bytes(result, &bytes[12]);
            return result;
        }
    }

    Mid360Driver::Mid360Driver(asio::io_context &io_context,
                               const asio::ip::address &host_ip,
                               const std::function<void(const asio::ip::address &lidar_ip, const std::vector<Point> &points)> &on_receive_pointcloud,
                               const std::function<void(const asio::ip::address &lidar_ip, const ImuMsg &imu_msg)> &on_receive_imu)
        : host_ip(host_ip),
          receive_pointcloud_socket(io_context),
          receive_imu_socket(io_context),
          on_receive_pointcloud(on_receive_pointcloud),
          on_receive_imu(on_receive_imu) {
        receive_pointcloud_socket.open(asio::ip::udp::v4());
        receive_pointcloud_socket.bind(asio::ip::udp::endpoint(host_ip, 56301));
        receive_imu_socket.open(asio::ip::udp::v4());
        receive_imu_socket.bind(asio::ip::udp::endpoint(host_ip, 56401));
        co_spawn(io_context, receive_pointcloud(), asio::detached);
        co_spawn(io_context, receive_imu(), asio::detached);
    }

    Mid360Driver::~Mid360Driver() {
        stop();
    }

    void Mid360Driver::stop() {
        is_running.store(false, std::memory_order_relaxed);
        asio::error_code error_code;
        receive_pointcloud_socket.close(error_code);
        receive_imu_socket.close(error_code);
    }

    asio::awaitable<void> Mid360Driver::receive_pointcloud() {
        uint8_t buffer[1400];
        asio::ip::udp::endpoint sender_endpoint;

        auto executor = co_await asio::this_coro::executor;

        while (is_running.load(std::memory_order_relaxed)) {
            asio::error_code ec;

            co_await receive_pointcloud_socket.async_receive_from(
                    asio::buffer(buffer),
                    sender_endpoint,
                    asio::redirect_error(asio::use_awaitable, ec));

            if (!is_running.load(std::memory_order_relaxed))
                co_return;

            if (ec) [[unlikely]] {
                // socket 异常 → 重建
                asio::error_code ignore;
                receive_pointcloud_socket.close(ignore);
                receive_pointcloud_socket.open(asio::ip::udp::v4(), ignore);
                receive_pointcloud_socket.bind(
                        asio::ip::udp::endpoint(host_ip, POINTCLOUD_RECV_PORT), ignore);

                // 防止忙等
                asio::steady_timer timer(executor, RESTART_DELAY);
                co_await timer.async_wait(asio::use_awaitable);
                continue;
            }

            if (sender_endpoint.port() != POINTCLOUD_SEND_PORT)
                continue;

            const auto &header =
                    *reinterpret_cast<const DataHeader *>(buffer);

            double header_timestamp =
                    static_cast<double>(header.timestamp) * 1e-9;

            if (header.time_type == TimestampType::kTimestampTypeNoSync) {
                std::lock_guard<std::mutex> lock(delta_time_mutex_);
                auto [iter, inserted] =
                        delta_time_map.try_emplace(sender_endpoint.address());
                if (inserted) {
                    double now =
                            static_cast<double>(
                                    std::chrono::high_resolution_clock::now()
                                            .time_since_epoch()
                                            .count()) *
                            1e-9;
                    iter->second = now - header_timestamp;
                    header_timestamp = now;
                } else {
                    header_timestamp += iter->second;
                }
            }

            points.clear();
            points.reserve(header.dot_num);

            if (header.data_type ==
                DataType::kLivoxLidarCartesianCoordinateHighData) {
                const auto *raw_points =
                        reinterpret_cast<const CartesianHighPoint *>(
                                buffer + sizeof(DataHeader));
                for (size_t i = 0; i < header.dot_num; ++i) {
                    const auto &rp = raw_points[i];
                    if ((rp.tag & 0b00111111) != 0)
                        continue;

                    Point p;
                    p.timestamp =
                            header_timestamp +
                            static_cast<double>(header.time_interval * i) /
                                    header.dot_num * 1e-10;
                    p.x = rp.x * 0.001f;
                    p.y = rp.y * 0.001f;
                    p.z = rp.z * 0.001f;
                    p.intensity = rp.reflectivity;
                    points.push_back(p);
                }
            } else if (header.data_type ==
                       DataType::kLivoxLidarCartesianCoordinateLowData) {
                const auto *raw_points =
                        reinterpret_cast<const CartesianLowPoint *>(
                                buffer + sizeof(DataHeader));
                for (size_t i = 0; i < header.dot_num; ++i) {
                    const auto &rp = raw_points[i];
                    if ((rp.tag & 0b00111111) != 0)
                        continue;

                    Point p;
                    p.timestamp =
                            header_timestamp +
                            static_cast<double>(header.time_interval * i) /
                                    header.dot_num * 1e-10;
                    p.x = rp.x * 0.001f;
                    p.y = rp.y * 0.001f;
                    p.z = rp.z * 0.001f;
                    p.intensity = rp.reflectivity;
                    points.push_back(p);
                }
            } else if (header.data_type ==
                       DataType::kLivoxLidarSphericalCoordinateData) {
                const auto *raw_points =
                        reinterpret_cast<const SphericalPoint *>(
                                buffer + sizeof(DataHeader));
                for (size_t i = 0; i < header.dot_num; ++i) {
                    const auto &rp = raw_points[i];
                    if ((rp.tag & 0b00111111) != 0)
                        continue;

                    double r = rp.depth / 1000.0;
                    double theta = rp.theta / 100.0 / 180.0 * M_PI;
                    double phi = rp.phi / 100.0 / 180.0 * M_PI;

                    Point p;
                    p.x = static_cast<float>(r * sin(theta) * cos(phi));
                    p.y = static_cast<float>(r * sin(theta) * sin(phi));
                    p.z = static_cast<float>(r * cos(theta));
                    p.intensity = rp.reflectivity;
                    points.push_back(p);
                }
            }

            on_receive_pointcloud(sender_endpoint.address(), points);
        }
    }


    asio::awaitable<void> Mid360Driver::receive_imu() {
        uint8_t buffer[1400];
        asio::ip::udp::endpoint sender_endpoint;

        auto executor = co_await asio::this_coro::executor;

        while (is_running.load(std::memory_order_relaxed)) {
            asio::error_code ec;

            co_await receive_imu_socket.async_receive_from(
                    asio::buffer(buffer),
                    sender_endpoint,
                    asio::redirect_error(asio::use_awaitable, ec));

            if (!is_running.load(std::memory_order_relaxed))
                co_return;

            if (ec) [[unlikely]] {
                asio::error_code ignore;
                receive_imu_socket.close(ignore);
                receive_imu_socket.open(asio::ip::udp::v4(), ignore);
                receive_imu_socket.bind(
                        asio::ip::udp::endpoint(host_ip, IMU_RECV_PORT), ignore);

                asio::steady_timer timer(executor, RESTART_DELAY);
                co_await timer.async_wait(asio::use_awaitable);
                continue;
            }

            if (sender_endpoint.port() != IMU_SEND_PORT)
                continue;

            const auto &header =
                    *reinterpret_cast<const DataHeader *>(buffer);

            double header_timestamp =
                    static_cast<double>(header.timestamp) * 1e-9;

            if (header.time_type == TimestampType::kTimestampTypeNoSync) {
                std::lock_guard<std::mutex> lock(delta_time_mutex_);
                auto [iter, inserted] =
                        delta_time_map.try_emplace(sender_endpoint.address());
                if (inserted) {
                    double now =
                            static_cast<double>(
                                    std::chrono::high_resolution_clock::now()
                                            .time_since_epoch()
                                            .count()) *
                            1e-9;
                    iter->second = now - header_timestamp;
                    header_timestamp = now;
                } else {
                    header_timestamp += iter->second;
                }
            }

            const auto &raw_imu =
                    *reinterpret_cast<const Imu *>(buffer + sizeof(DataHeader));

            ImuMsg imu;
            imu.timestamp = header_timestamp;
            imu.angular_velocity_x = raw_imu.angular_velocity_x;
            imu.angular_velocity_y = raw_imu.angular_velocity_y;
            imu.angular_velocity_z = raw_imu.angular_velocity_z;
            imu.linear_acceleration_x = raw_imu.linear_acceleration_x;
            imu.linear_acceleration_y = raw_imu.linear_acceleration_y;
            imu.linear_acceleration_z = raw_imu.linear_acceleration_z;

            on_receive_imu(sender_endpoint.address(), imu);
        }
    }


}// namespace mid360_driver
