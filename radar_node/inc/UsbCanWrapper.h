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
#include "usbcan/controlcan/controlcan.h"
#include "GlobalDefines.h"

namespace radar{

/* chuang-xin-ke-ji CANalyst-II */
class UsbCanClassical
{
private:
    constexpr static unsigned int dev_idx_ = 0;
    constexpr static unsigned int BUF_MAX_ = 1000;
    unsigned int can_chn_;
    VCI_CAN_OBJ* buffer_;

public:
    UsbCanClassical(const std::string can_name){
        can_chn_ = std::stoi(can_name);
        assert(open_usbcan());
    }

    ~UsbCanClassical() {
        close_usbcan();
    }

    bool open_usbcan(){
        /* suppose we have at most 5 device */
        VCI_BOARD_INFO infos[5];
        int num = VCI_FindUsbDevice2(infos);
        ROS_INFO("find %d usbcan device", num);
        /* we only accept 1 device */
        assert(num == 1);

        /* open device 0 */
        VCI_BOARD_INFO info;
        if(VCI_OpenDevice(VCI_USBCAN2, 0, 0) == 1 &&
           VCI_ReadBoardInfo(VCI_USBCAN2, 0, &info) == 1)
        {
            std::string sn(info.str_Serial_Num, 20);
            std::string hw(info.str_hw_Type, 10);
            ROS_INFO("open device %d success, sn: %s, hw: %s, firmware: %d", dev_idx_, sn.c_str(), hw.c_str(), info.fw_Version);
        }else{
            ROS_ERROR("open device %d error!", dev_idx_);
            return false;
        }

        VCI_INIT_CONFIG config;
        /* filter none */
        config.AccCode = 0;
        config.AccMask = 0xFFFFFFFF;
        config.Filter = 1;
        /* baudrate = 500Kbps */
        config.Timing0 = 0x00;
        config.Timing1 = 0x1C;
        config.Mode = 0;

        if(VCI_InitCAN(VCI_USBCAN2, dev_idx_, can_chn_, &config) != 1){
            ROS_ERROR("init can device %d channel %d error!", dev_idx_, can_chn_);
            VCI_CloseDevice(VCI_USBCAN2, dev_idx_);
        }

        if(VCI_StartCAN(VCI_USBCAN2, dev_idx_, can_chn_) != 1){
            ROS_ERROR("start can device %d channel %d error!", dev_idx_, can_chn_);
            VCI_CloseDevice(VCI_USBCAN2, dev_idx_);
        }

        ROS_INFO("start can device %d channel %d, baudrate: 500Kbps", dev_idx_, can_chn_);
        buffer_ = new VCI_CAN_OBJ[BUF_MAX_];

        return true;
    }

    void close_usbcan(){
        VCI_ResetCAN(VCI_USBCAN2, dev_idx_, can_chn_);
        VCI_CloseDevice(VCI_USBCAN2, dev_idx_);
        delete[] buffer_;
    }

    int send_frame(const CanFrameClassical_t& frame){
        VCI_CAN_OBJ send[1];
        send[0].ID = frame.can_id;
        send[0].SendType = 0;
        send[0].RemoteFlag = 0;
        /* standard frame */
        send[0].ExternFlag = 0;
        send[0].DataLen = frame.can_dlc;

        memcpy(send[0].Data, frame.data, frame.can_dlc);
        // for(int i=0; i<frame.can_dlc; i++){
        //     send[0].Data[i] = frame.data[i];
        // }

        if(VCI_Transmit(VCI_USBCAN2, dev_idx_, can_chn_, send, 1) == 1){
            ROS_INFO("send frame %x success", frame.can_id);
            return frame.can_dlc;
        }else{
            ROS_ERROR("send frame %x fail", frame.can_id);
        }

        return -1;
    }

    int recv_frame(FrameList_t& frames){
        /* WaitTime = 100ms? */
        int reclen = VCI_Receive(VCI_USBCAN2, dev_idx_, can_chn_, buffer_, BUF_MAX_, 100);
        for(int i=0; i<reclen; i++){
            CanFrameClassical_t frame;
            frame.can_id = buffer_[i].ID;
            frame.can_dlc = buffer_[i].DataLen;
            memcpy(frame.data, buffer_[i].Data, frame.can_dlc);
            frames.push(frame);
            // ROS_INFO("recv canid: 0x%03X, len: %d", frame.can_id, frame.can_dlc);
        }

        return reclen;
    }

};

}