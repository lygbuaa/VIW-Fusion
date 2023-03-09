#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <errno.h>
#include <ros/ros.h>
#include "GlobalDefines.h"

namespace radar{
class SocketCanClassical
{
private:
    std::string can_name_;
    struct sockaddr_can addr_;
    struct ifreq ifr_;
    int fd_ = -1;

public:
    SocketCanClassical(const std::string can_name){
        can_name_ = can_name;
        assert(open_socket());
    }

    ~SocketCanClassical() {
        close_socket();
    }

    bool open_socket(){
        if ((fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
            ROS_ERROR("create can socket fail: %s", strerror(fd_));
            return false;
        }

        strcpy(ifr_.ifr_name, can_name_.c_str());
        ioctl(fd_, SIOCGIFINDEX, &ifr_);

        memset(&addr_, 0, sizeof(addr_));
        addr_.can_family = AF_CAN;
        addr_.can_ifindex = ifr_.ifr_ifindex;

        struct timeval tv;
        tv.tv_sec = 1;  /* Timeout */
        tv.tv_usec = 0;
        setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(struct timeval));

        int rc = ::bind(fd_, (struct sockaddr *)&addr_, sizeof(addr_));

        if (rc < 0) {
            ROS_ERROR("bind file descriptor fail: %s", strerror(rc));
            return false;
	    }

        return true;
    }

    void close_socket(){
        if(fd_ >= 0){
            int rc = close(fd_);
            if(rc < 0){
                ROS_ERROR("close socket fail: %s", strerror(rc));
            }
            fd_ = -1;
        }
    }

    int send_frame(const CanFrameClassical_t& frame){
        if (write(fd_, &frame, sizeof(CanFrameClassical_t)) != sizeof(CanFrameClassical_t)) {
            ROS_ERROR("write socket fail: %d", frame.can_dlc);
            return -1;
        }
        return frame.can_dlc;
    }

    int recv_frame(CanFrameClassical_t& frame){
        int nbytes = read(fd_, &frame, sizeof(CanFrameClassical_t));

        if (nbytes < 0) {
            ROS_ERROR("read socket fail: %s", strerror(nbytes));
            return nbytes;
        }

        // ROS_INFO("recv canid: 0x%03X, len: %d", frame.can_id, frame.can_dlc);
#if 0
        for (int i = 0; i < frame.can_dlc; i++){
            ROS_INFO("data[%d] = %02X ", i, frame.data[i]);
        }
#endif
        return nbytes;
    }

    int recv_frame(FrameList_t& frames){
        CanFrameClassical_t frame;
        int nbytes = recv_frame(frame);
        if(nbytes > 0){
            frames.push(frame);
        }
        return nbytes;
    }


};

}