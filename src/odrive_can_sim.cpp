#include <cstring>
#include <string>
#include <memory>
#include <mutex>
#include <thread>
#include <atomic>
#include <array>
#include <chrono>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <algorithm>
#include <iostream>

#include "odrive_can_sim.hpp"

namespace odrive_can_sim
{
    /**
     * @brief Construct a new Dbot Can object
     *
     */
    OdriveCanSim::OdriveCanSim()
    {
        // Initialize
        can_name_ = "vcan0";
        joint_can_ids_ = {0x001, 0x002, 0x003, 0x004, 0x005, 0x006};

        // Zero
        encoder_positions_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        encoder_velocities_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        prev_encoder_pos_ = encoder_positions_;

        // Threads
        encoder_broadcast_ms_ = std::chrono::milliseconds(10);
        encoder_velocity_update_ms_ = std::chrono::milliseconds(10);
    }

    /**
     * @brief Construct a new Odrive Can Sim object
     *
     * @param can_name
     * @param joint_can_ids
     * @param encoder_broadcast_ms
     */
    OdriveCanSim::OdriveCanSim(const std::string &can_name, const std::array<int, 6> &joint_can_ids, int encoder_broadcast_ms = 10)
    {
        // Initialize
        can_name_ = can_name;
        joint_can_ids_ = joint_can_ids;

        // Zero
        encoder_positions_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        encoder_velocities_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        prev_encoder_pos_ = encoder_positions_;

        // Threads
        encoder_broadcast_ms_ = std::chrono::milliseconds(encoder_broadcast_ms);
        encoder_velocity_update_ms_ = std::chrono::milliseconds(10);
    }

    /**
     * @brief Initializes the Can Bus using linux's built in SocketCan
     *
     */
    bool OdriveCanSim::initialize()
    {
        return true;
    }

    /**
     * @brief Creates a connection to the CAN bus
     *
     * @return true if successful, false otherwise
     */
    bool OdriveCanSim::connect()
    {
        printf("Odrive setting the sockets\r\n");
        // Set Socket
        socket_read_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (socket_read_ < 0)
        {
            socket_read_ = 0;
            socket_write_ = 0;
            return false;
        }

        socket_write_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (socket_write_ < 0)
        {
            socket_read_ = 0;
            socket_write_ = 0;
            return false;
        }

        // Specify can_interface device
        printf("Odrive specify can_interface device\r\n");
        int ret_r;
        struct ifreq ifr_read;
        strcpy(ifr_read.ifr_name, can_name_.c_str());
        ret_r = ioctl(socket_read_, SIOCGIFINDEX, &ifr_read);
        if (ret_r < 0)
        {
            socket_read_ = 0;
            socket_write_ = 0;
            return false;
        }

        int ret_w;
        struct ifreq ifr_write;
        strcpy(ifr_write.ifr_name, can_name_.c_str());
        ret_w = ioctl(socket_write_, SIOCGIFINDEX, &ifr_write);
        if (ret_w < 0)
        {
            socket_read_ = 0;
            socket_write_ = 0;
            return false;
        }

        // Bind the socket to interface
        printf("Odrive bind the socket to interface\r\n");
        struct sockaddr_can addr_read;
        addr_read.can_family = AF_CAN;
        addr_read.can_ifindex = ifr_read.ifr_ifindex;
        ret_r = bind(socket_read_, (struct sockaddr *)&addr_read, sizeof(addr_read));
        if (ret_r < 0)
        {
            socket_read_ = 0;
            socket_write_ = 0;
            return false;
        }

        struct sockaddr_can addr_write;
        addr_write.can_family = AF_CAN;
        addr_write.can_ifindex = ifr_write.ifr_ifindex;
        ret_w = bind(socket_write_, (struct sockaddr *)&addr_write, sizeof(addr_write));
        if (ret_w < 0)
        {
            socket_read_ = 0;
            socket_write_ = 0;
            return false;
        }

        // Filtering rules
        // struct can_filter rfilter[1];
        // rfilter[0].can_id = 0x123;
        // rfilter[0].can_mask = CAN_SFF_MASK;
        // setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));
        setsockopt(socket_write_, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

        // Start the Read and write thread
        printf("Odrive read/write threads starting\r\n");
        is_can_reading_.store(true);
        is_can_writing_.store(true);
        is_encoder_velocity_updater_running_ = true;
        can_read_thread_ = std::thread{&OdriveCanSim::can_read_task, this};
        can_write_thread_ = std::thread{&OdriveCanSim::can_write_task, this};
        encoder_velocity_updater_thread_ = std::thread{&OdriveCanSim::encoder_velocity_updater, this};

        printf("Odrive CAN connected\r\n");
        // Thread synchronization flags
        is_main_running_ = true;
        return true;
    }

    /**
     * @brief Disconnects from the CAN bus
     *
     * @return true if successful, false otherwise
     */
    bool OdriveCanSim::disconnect()
    {
        // Disable CAN reading and stop the threads
        is_can_reading_.store(false);
        is_can_writing_.store(false);
        is_encoder_velocity_updater_running_ = false;

        // Stop spinning
        is_main_running_ = false;

        // Close sockets
        int ret_r = close(socket_read_);
        int ret_w = close(socket_write_);
        return (ret_r >= 0) && (ret_w >= 0);
    }

    /**
     * @brief Get the encoder positions
     *
     * @return std::array<float, 6>
     */
    std::array<float, 6> OdriveCanSim::get_position()
    {
        const std::lock_guard<std::mutex> lock(mtx_pos_);
        return encoder_positions_;
    }

    /**
     * @brief Get the encoder velocities
     *
     * @return std::array<float, 6>
     */
    std::array<float, 6> OdriveCanSim::get_velocity()
    {
        const std::lock_guard<std::mutex> lock(mtx_vel_);
        return encoder_velocities_;
    }

    /**
     * @brief Set the encoder positions
     *
     * @param pos
     * @return true if successful, false otherwise
     */
    void OdriveCanSim::set_position(const std::array<float, 6> &encs)
    {
        const std::lock_guard<std::mutex> lock(mtx_pos_);
        encoder_positions_ = encs;
    }

    /**
     * @brief Set the encoder positions
     *
     * @param enc_pos
     * @param idx
     */
    void OdriveCanSim::set_position(float enc_pos, int idx)
    {
        // Set
        const std::lock_guard<std::mutex> lock(mtx_pos_);
        encoder_positions_[idx] = enc_pos;
    }

    /**
     * @brief Set the encoder velocity
     *
     * @param encs
     */
    void OdriveCanSim::set_velocity(const std::array<float, 6> &encs)
    {
        // Set
        const std::lock_guard<std::mutex> lock(mtx_vel_);
        encoder_velocities_ = encs;
    }

    /**
     * @brief Set the encoder velocity
     *
     * @param pos
     * @return true if successful, false otherwise
     */
    void OdriveCanSim::set_velocity(float enc_vel, int idx)
    {
        // Set
        const std::lock_guard<std::mutex> lock(mtx_vel_);
        encoder_velocities_[idx] = enc_vel;
    }

    /**
     * @brief Infinite looping
     */
    void OdriveCanSim::spin()
    {
        printf("Odrive CAN spinning\r\n");
        printf("Send a 0xFFF CAN id to stop\r\n");

        // Start Velocity updater
        main_velocity_update_semaphore_.release();

        // Start Read/Write
        main_can_read_semaphore_.release();
        main_can_write_semaphore_.release();

        while (true)
        {
            if (!is_main_running_)
                break;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        printf("Odrive waiting for threads to join\r\n");
        can_read_thread_.join();
        can_write_thread_.join();
        encoder_velocity_updater_thread_.join();
        printf("Odrive threads joined\r\n");
        printf("Odrive CAN spinning ended\r\n");
    }

    /**
     * @brief Get the node id object
     *
     * @param msg_id
     * @return int
     */
    int OdriveCanSim::get_node_id(int msg_id)
    {
        return (msg_id >> 5);
    }

    /**
     * @brief Get the command id object
     *
     * @param msg_id
     * @return int
     */
    int OdriveCanSim::get_command_id(int msg_id)
    {
        return (msg_id & 0x01F);
    }

    /**
     * @brief CAN bus read task
     *
     */
    void OdriveCanSim::can_read_task()
    {
        // Set
        struct can_frame frame;
        memset(&frame, 0, sizeof(struct can_frame));

        // Wait for the main thread
        main_can_read_semaphore_.acquire();

        std::cout << "CAN read starting\r\n";
        while (true)
        {
            // Break Guard
            if (!is_can_reading_.load())
                break;

            // Read
            // This is a blocking function, it waits for an available CAN frame in the buffer
            int nbytes = read(socket_read_, &frame, sizeof(frame));

            // Guard
            if (nbytes < 0)
                continue;

            // Handle CAN message
            can_handle_message(frame);
        }

        std::cout << "CAN read ended\r\n";
    }

    /**
     * @brief CAN bus write task
     *
     */
    void OdriveCanSim::can_write_task()
    {
        // Set
        struct can_frame frame;
        memset(&frame, 0, sizeof(struct can_frame));

        // Wait for the main thread
        main_can_write_semaphore_.acquire();

        // printf("CAN write starting");
        std::cout << "CAN write starting\r\n";
        while (true)
        {
            // Break Guard
            if (!is_can_writing_.load())
                break;

            // Frame
            auto enc_pos = get_position();
            auto enc_vel = get_velocity();
            for (size_t i = 0; i < enc_pos.size(); i++)
            {
                // Axis and data
                int axis_id = joint_can_ids_[i];
                float pos = enc_pos[i];
                float vel = enc_vel[i];

                // CAN send [0x009] encoder estimates
                struct can_frame frame;
                int command_id = 0x009;
                uint8_t data[8];
                std::memcpy(&data[0], &pos, 4);
                std::memcpy(&data[4], &vel, 4);
                frame.can_id = (axis_id << 5) | (command_id);
                frame.len = 8;
                std::memcpy(frame.data, data, sizeof(data));

                int nbytes = write(socket_write_, &frame, sizeof(frame));
                if (nbytes < 0)
                    continue;
            }

            // Write at 100Hz [10ms]
            std::this_thread::sleep_for(encoder_broadcast_ms_);
        }

        // printf("CAN write ended");
        std::cout << "CAN write ended\r\n";
    }

    /**
     * @brief Handle a CAN message frame
     *
     * @param frame
     */
    void OdriveCanSim::can_handle_message(const struct can_frame &frame)
    {
        // Process ID's
        int msg_id = frame.can_id;
        int node_id = get_node_id(msg_id);
        int command_id = get_command_id(msg_id);

        // Global guard
        if (msg_id == 0xFFF)
        {
            printf("\r\n");
            printf("Stopping everything [OxFFF]\r\n");
            // Stop Everything
            stop_callback(frame);
        }

        // Guard
        // Check if 'node_id' is inside 'joint_can_ids_'
        bool is_joint_can_id = std::find(joint_can_ids_.begin(), joint_can_ids_.end(), node_id) != joint_can_ids_.end();
        if (!is_joint_can_id)
            return;

        // Node Id is inside the container
        switch (command_id)
        {
        case 0x001:
            /* Heartbeat*/
            break;
        case 0x00C:
            // Set Input Pos
            set_input_pos_callback(frame);
            break;
        default:
            break;
        }
    }

    /**
     * @brief Handles the set input pos
     *
     * @param frame
     */
    void OdriveCanSim::set_input_pos_callback(const struct can_frame &frame)
    {
        // ID's
        int msg_id = frame.can_id;
        int node_id = get_node_id(msg_id);

        // Get index
        auto target_itr = std::find(joint_can_ids_.begin(), joint_can_ids_.end(), node_id);
        int joint_idx = std::distance(joint_can_ids_.begin(), target_itr);

        // Process Data
        // 0  1   2  3        4  5  6  7
        // [] [] [] []   --   [] [] [] []
        //   4 bytes     --     4 bytes
        // encoder pos   --   encoder vel
        float enc_pos;
        std::memcpy(&enc_pos, &frame.data[0], 4);

        // Thread safety
        // Position, Velocity
        set_position(enc_pos, joint_idx);
    }

    /**
     * @brief Handles the stop command
     *
     * @param frame
     */
    void OdriveCanSim::stop_callback(const struct can_frame &frame)
    {
        (void)frame;
        this->disconnect();
    }

    /**
     * @brief
     *
     */
    void OdriveCanSim::encoder_velocity_updater()
    {
        // Wait for the main thread
        main_velocity_update_semaphore_.acquire();

        printf("Velocity updater starting\r\n");

        // Initialize
        prev_time_ = std::chrono::steady_clock::now();
        prev_encoder_pos_ = get_position();
        auto curr_vel = get_velocity();
        float epsilon = 0.001f;

        while (true)
        {
            // Guard
            if (!is_encoder_velocity_updater_running_)
                break;

            // Get
            auto curr_pos = get_position();
            auto curr_time = std::chrono::steady_clock::now();
            std::chrono::duration<float> dt = curr_time - prev_time_;
            if (std::abs(dt.count()) < epsilon)
                continue;

            // Set
            for (size_t i = 0; i < curr_pos.size(); i++)
            {
                curr_vel[i] = (curr_pos[i] - prev_encoder_pos_[i]) / dt.count();
            }
            set_velocity(curr_vel);

            prev_time_ = curr_time;
            prev_encoder_pos_ = curr_pos;
            std::this_thread::sleep_for(encoder_velocity_update_ms_);
        }

        printf("Velocity updater end\r\n");
    }
}