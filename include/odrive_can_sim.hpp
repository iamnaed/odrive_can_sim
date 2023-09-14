#ifndef ODRIVE_CAN_SIM__ODRIVE_CAN_SIM_HPP_
#define ODRIVE_CAN_SIM__ODRIVE_CAN_SIM_HPP_

#include <string>
#include <mutex>
#include <thread>
#include <atomic>
#include <array>
#include <chrono>
#include <linux/can.h>
#include <linux/can/raw.h>

namespace odrive_can_sim
{
    class OdriveCanSim
    {
    public:
        /**
         * @brief Construct a new Dbot Can object
         *
         */
        OdriveCanSim();

        /**
         * @brief Construct a new Odrive Can Sim object
         *
         * @param can_name
         * @param joint_can_ids
         * @param encoder_broadcast_ms
         */
        OdriveCanSim(const std::string &can_name, const std::array<int, 6> &joint_can_ids, int encoder_broadcast_ms);

        /**
         * @brief Initializes the Can Bus using linux's built in SocketCan
         *
         */
        bool initialize();

        /**
         * @brief Creates a connection to the CAN bus
         *
         * @return true if successful, false otherwise
         */
        bool connect();

        /**
         * @brief Disconnects from the CAN bus
         *
         * @return true if successful, false otherwise
         */
        bool disconnect();

        /**
         * @brief Get the encoder positions
         *
         * @return std::array<float, 6>
         */
        std::array<float, 6> get_position();

        /**
         * @brief Get the encoder velocities
         *
         * @return std::array<float, 6>
         */
        std::array<float, 6> get_velocity();

        /**
         * @brief Set the encoder positions
         *
         * @param pos
         * @return true if successful, false otherwise
         */
        void set_position(const std::array<float, 6> &encs);
        
        /**
         * @brief Set the encoder positions
         *
         * @param enc_pos
         * @param idx
         */
        void set_position(float enc_pos, int idx);

        /**
         * @brief Set the encoder velocity
         *
         * @param encs
         */
        void set_velocity(const std::array<float, 6> &encs);

        /**
         * @brief Set the encoder velocity
         *
         * @param pos
         * @return true if successful, false otherwise
         */
        void set_velocity(float enc_vel, int idx);

        /**
         * @brief Infinite looping
         */
        void spin();

    private:
        /**
         * @brief Get the node id object
         *
         * @param msg_id
         * @return int
         */
        int get_node_id(int msg_id);

        /**
         * @brief Get the command id object
         *
         * @param msg_id
         * @return int
         */
        int get_command_id(int msg_id);

        /**
         * @brief CAN bus read task
         *
         */
        void can_read_task();

        /**
         * @brief CAN bus write task
         *
         */
        void can_write_task();

        /**
         * @brief Handle a CAN message frame
         *
         * @param frame
         */
        void can_handle_message(const struct can_frame &frame);

        /**
         * @brief Handles the set input pos
         *
         * @param frame
         */
        void set_input_pos_callback(const struct can_frame &frame);

        /**
         * @brief Handles the stop command
         *
         * @param frame
         */
        void stop_callback(const struct can_frame &frame);

        /**
         * @brief
         *
         */
        void encoder_velocity_updater();

    private:
        /**
         * @brief Internal members
         *
         */
        std::string can_name_;
        std::array<int, 6> joint_can_ids_;
        std::array<float, 6> encoder_positions_;
        std::array<float, 6> encoder_velocities_;
        std::array<float, 6> prev_encoder_pos_;
        std::chrono::steady_clock::time_point prev_time_;

        /**
         * @brief Socket handles for CAN communication
         *
         */
        int socket_read_;
        int socket_write_;

        /**
         * @brief For multithreading members
         *
         */
        std::thread can_read_thread_;
        std::thread can_write_thread_;
        std::thread encoder_velocity_updater_thread_;
        std::atomic<bool> is_can_reading_;
        std::atomic<bool> is_can_writing_;
        std::atomic<bool> is_main_running_;
        std::atomic<bool> is_encoder_velocity_updater_running_;
        std::mutex mtx_pos_;
        std::mutex mtx_vel_;

        /**
         * @brief Thread sync flags
         */
        std::binary_semaphore main_can_read_semaphore_{0};
        std::binary_semaphore main_can_write_semaphore_{0};
        std::binary_semaphore main_velocity_update_semaphore_{0};

        /**
         * @brief Timers
         *
         */
        std::chrono::milliseconds encoder_broadcast_ms_;
        std::chrono::milliseconds encoder_velocity_update_ms_;
    };
}

#endif // ODRIVE_CAN_SIM__ODRIVE_CAN_SIM_HPP_