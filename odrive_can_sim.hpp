#ifndef ODRIVE_CAN_SIM__ODRIVE_CAN_SIM_HPP_
#define ODRIVE_CAN_SIM__ODRIVE_CAN_SIM_HPP_

#include <cstring>
#include <string>
#include <memory>
#include <mutex>
#include <thread>
#include <atomic>
#include <array>

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

class Joint
{
    static const int J0 = 0;
    static const int J1 = 1;
    static const int J2 = 2;
    static const int J3 = 3;
    static const int J4 = 4;
    static const int J5 = 5;
};

class OdriveCanSim
{
public:
    /**
     * @brief Construct a new Dbot Can object
     * 
     */
    OdriveCanSim()
    {
        // Initialize
        can_name_ = "vcan0";
        joint_can_ids_[0] = 0x001;
        joint_can_ids_[1] = 0x002;
        joint_can_ids_[2] = 0x003;
        joint_can_ids_[3] = 0x004;
        joint_can_ids_[4] = 0x005;
        joint_can_ids_[5] = 0x006;

        // Zero
        encoder_positions_ = {0.0,0.0,0.0,0.0,0.0,0.0};
        encoder_velocities_ = {0.0,0.0,0.0,0.0,0.0,0.0};
        prev_encoder_pos_ = encoder_positions_;
    }

    /**
     * @brief Initializes the Can Bus using linux's built in SocketCan
     * 
     */
    bool initialize()
    {
        return true;
    }

    /**
     * @brief Creates a connection to the CAN bus 
     * 
     * @return true if successful, false otherwise
     */
    bool connect()
    {
        printf("Odrive setting the sockets\r\n");    
        // Set Socket
        socket_read_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if(socket_read_ < 0)
        {
            socket_read_ = 0;
            socket_write_ = 0;
            return false;
        }

        socket_write_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if(socket_write_ < 0)
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
        if (ret_r < 0) {
            socket_read_ = 0;
            socket_write_ = 0;
            return false;
        }

        int ret_w;
        struct ifreq ifr_write;
        strcpy(ifr_write.ifr_name, can_name_.c_str());
        ret_w = ioctl(socket_write_, SIOCGIFINDEX, &ifr_write);
        if (ret_w < 0) {
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
        if (ret_r < 0) {
            socket_read_ = 0;
            socket_write_ = 0;
            return false;
        }

        struct sockaddr_can addr_write;
        addr_write.can_family = AF_CAN;
        addr_write.can_ifindex = ifr_write.ifr_ifindex;
        ret_w = bind(socket_write_, (struct sockaddr *)&addr_write, sizeof(addr_write));
        if (ret_w < 0) {
            socket_read_ = 0;
            socket_write_ = 0;
            return false;
        }
        
        // Filtering rules
        //struct can_filter rfilter[1];
        //rfilter[0].can_id = 0x123;
        //rfilter[0].can_mask = CAN_SFF_MASK;
        //setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));
        setsockopt(socket_write_, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);

        // Start the Read and write thread
        printf("Odrive read/write threads starting\r\n");    
        is_can_reading_.store(true);
        is_can_writing_.store(true);
        can_read_thread_ = std::thread{&OdriveCanSim::can_read_task, this};
        can_write_thread_ = std::thread{&OdriveCanSim::can_write_task, this};
          
        // Starting velocity updater
        is_velocity_updating_.store(true);
        //velocity_updater_thread_ = std::thread{&OdriveCanSim::velocity_updater_task, this};

        printf("Odrive CAN connected\r\n");  
        is_main_running_ = true;
        return true;
    }

    /**
     * @brief Disconnects from the CAN bus
     * 
     * @return true if successful, false otherwise
     */
    bool disconnect()
    {
        // Disable CAN reading and stop the threads
        is_can_reading_.store(false);
        is_can_writing_.store(false);
        is_velocity_updating_.store(false);

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
    std::array<float, 6> get_position()
    {
        float enc0, enc1, enc2, enc3, enc4, enc5;

        mtx_pos_.lock();
        enc0 = encoder_positions_[0];
        enc1 = encoder_positions_[1];
        enc2 = encoder_positions_[2];
        enc3 = encoder_positions_[3];
        enc4 = encoder_positions_[4];
        enc5 = encoder_positions_[5];
        mtx_pos_.unlock();

        return std::array<float, 6>{enc0, enc1, enc2, enc3, enc4, enc5};
    }

    /**
     * @brief Get the encoder velocities
     * 
     * @return std::array<float, 6> 
     */
    std::array<float, 6> get_velocity()
    {
        float enc0, enc1, enc2, enc3, enc4, enc5;

        mtx_vel_.lock();
        enc0 = encoder_velocities_[0];
        enc1 = encoder_velocities_[1];
        enc2 = encoder_velocities_[2];
        enc3 = encoder_velocities_[3];
        enc4 = encoder_velocities_[4];
        enc5 = encoder_velocities_[5];
        mtx_vel_.unlock();

        return std::array<float, 6>{enc0, enc1, enc2, enc3, enc4, enc5};
    }

    /**
     * @brief Set the encoder positions
     * 
     * @param pos 
     * @return true if successful, false otherwise
     */
    bool set_position(std::array<float, 6> encs)
    {
        // Set
        mtx_pos_.lock();
        encoder_positions_[0] = encs[0];
        encoder_positions_[1] = encs[1];
        encoder_positions_[2] = encs[2];
        encoder_positions_[3] = encs[3];
        encoder_positions_[4] = encs[4];
        encoder_positions_[5] = encs[5];
        mtx_pos_.unlock();
        return true;
    }
    
    /**
     * @brief Set the encoder positions
     * 
     * @param pos 
     * @return true if successful, false otherwise
     */
    bool set_position(float enc_pos, int idx)
    {
        // Set
        mtx_pos_.lock();
        encoder_positions_[idx] = enc_pos;
        mtx_pos_.unlock();
        return true;
    }

    /**
     * @brief Set the encoder positions
     * 
     * @param pos 
     * @return true if successful, false otherwise
     */
    bool set_velocity(std::array<float, 6> encs)
    {
        // Set
        mtx_vel_.lock();
        encoder_velocities_[0] = encs[0];
        encoder_velocities_[1] = encs[1];
        encoder_velocities_[2] = encs[2];
        encoder_velocities_[3] = encs[3];
        encoder_velocities_[4] = encs[4];
        encoder_velocities_[5] = encs[5];
        mtx_vel_.unlock();
        return true;
    }
    
    /**
     * @brief Set the encoder velocity
     * 
     * @param pos 
     * @return true if successful, false otherwise
     */
    bool set_velocity(float enc_vel, int idx)
    {
        // Set
        mtx_vel_.lock();
        encoder_velocities_[idx] = enc_vel;
        mtx_vel_.unlock();
        return true;
    }
    
    /**
     * @brief Get errors in the odrive controllers
     * 
     * @return int 
     */
    int get_errors()
    {
        return 0x0000;
    }

    /**
     * @brief Clear errors in the odrive controllers
     * 
     * @return true if successful, false otherwise
     */
    bool clear_errors()
    {
        return true;
    }

    /**
     * @brief Inifite looping
    */
    void spin()
    {
        printf("Odrive CAN spinning\r\n");
        printf("Send a 0xFFF CAN id to stop\r\n");
        while(true)
        {
            if(!is_main_running_)
                break;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        printf("Odrive waiting for threads to join\r\n");
        can_read_thread_.join();
        can_write_thread_.join();
        printf("Odrive threads joined\r\n");
        printf("Odrive CAN spinning ended\r\n");
    }
    
private:
    /**
     * @brief Get the node id object
     * 
     * @param msg_id 
     * @return int 
     */
    int get_node_id(int msg_id)
    {
        return (msg_id >> 5);
    }

    /**
     * @brief Get the command id object
     * 
     * @param msg_id 
     * @return int 
     */
    int get_command_id(int msg_id)
    {
        return (msg_id & 0x01F);
    }

    /**
     * @brief CAN bus read task
     * 
     */
    void can_read_task()
    {
         // Set
        struct can_frame frame;
        memset(&frame, 0, sizeof(struct can_frame));

        std::cout << "CAN read starting\r\n"; 
        while(true)
        {
            // Break Guard
            if(!is_can_reading_.load())
                break;

            // Read
            // This is a blocking function, it waits for an available CAN frame in the buffer
            int nbytes = read(socket_read_, &frame, sizeof(frame));

            // Guard
            if(nbytes < 0)
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
    void can_write_task()
    {
         // Set
        struct can_frame frame;
        memset(&frame, 0, sizeof(struct can_frame));

        //printf("CAN write starting");
        std::cout << "CAN write starting\r\n"; 
        while(true)
        {
            // Break Guard
            if(!is_can_writing_.load())
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
                if(nbytes < 0)
                    continue;
            }

            // Write at 100Hz [10ms]
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        //printf("CAN write ended");
        std::cout << "CAN write ended\r\n"; 
    }

    /**
     * @brief Handle a CAN message frame
     * 
     * @param frame 
     */
    void can_handle_message(const struct can_frame& frame)
    {
        // Process ID's
        int msg_id = frame.can_id;
        int node_id = get_node_id(msg_id);
        int command_id = get_command_id(msg_id);

        // Global guard
        if(msg_id == 0xFFF)
        {
            printf("\r\n");
            printf("Stopping everything [OxFFF]\r\n");
            // Stop Everything
            stop_callback(frame);
        }

        // Guard
        // Check if 'node_id' is inside 'joint_can_ids_'
        bool is_joint_can_id = std::find(joint_can_ids_.begin(), joint_can_ids_.end(), node_id) != joint_can_ids_.end();
        if(!is_joint_can_id)
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
     */
    void velocity_updater_task()
    {
        std::cout << "Velocity update starting\r\n"; 
        while (true)
        {
            if(!is_velocity_updating_.load())
                break;

            std::array<float, 6> current_pos = get_position();
            std::array<float, 6> vel;
            for (size_t i = 0; i < current_pos.size(); i++)
            {
                // Updating at 100Hz
                // 10ms
                // 10/1000 s
                vel[0] = (current_pos[i] - prev_encoder_pos_[i]) * 100.0f;
            }
            
            set_velocity(vel);
            prev_encoder_pos_ = current_pos;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        std::cout << "Velocity update ended\r\n"; 
    }

    /**
     * @brief Handles the set input pos
     * 
     * @param frame 
     */
    void set_input_pos_callback(const struct can_frame& frame)
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
        float pos_buff;
        std::memcpy(&pos_buff, &frame.data[0], 4);

        // Thread safety
        // Position, Velocity
        set_position(pos_buff, joint_idx);
    }

    /**
     * @brief Handles the stop command
     * 
     * @param frame 
     */
    void stop_callback(const struct can_frame& frame)
    {
        this->disconnect();
    }

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
    std::thread velocity_updater_thread_;
    std::atomic<bool> is_can_reading_;
    std::atomic<bool> is_can_writing_;
    std::atomic<bool> is_velocity_updating_;
    std::mutex mtx_pos_;
    std::mutex mtx_vel_;
    bool is_main_running_;

};

#endif // ODRIVE_CAN_SIM__ODRIVE_CAN_SIM_HPP_