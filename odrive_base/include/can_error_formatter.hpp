#ifndef CAN_ERROR_FORMATTER_HPP
#define CAN_ERROR_FORMATTER_HPP

#include <linux/can.h>
#include <linux/can/error.h>
#include <string>
#include <sstream>

namespace can_utils {

std::string format_can_error_frame(const can_frame& frame) {
    std::ostringstream oss;
    bool first = true;

    auto tag = [&]() {
        if (!first) oss << ' ';
        first = false;
    };

    uint32_t err_id = frame.can_id & CAN_ERR_MASK;

    if (err_id & CAN_ERR_TX_TIMEOUT)   { tag(); oss << "TX_TIMEOUT"; }

    if (err_id & CAN_ERR_LOSTARB)
        tag(), oss << "LOSTARB[" << (int)frame.data[0] << "]";

    if (err_id & CAN_ERR_CRTL) {
        tag(), oss << "CTRL[";
        bool inner = true;
        auto item = [&](const char* s) {
            if (!inner) oss << ' ';
            oss << s;
            inner = false;
        };
        uint8_t c = frame.data[1];
        if (c & CAN_ERR_CRTL_RX_OVERFLOW)  item("RX_OVERFLOW");
        if (c & CAN_ERR_CRTL_TX_OVERFLOW)  item("TX_OVERFLOW");
        if (c & CAN_ERR_CRTL_RX_WARNING)   item("RX_WARNING");
        if (c & CAN_ERR_CRTL_TX_WARNING)   item("TX_WARNING");
        if (c & CAN_ERR_CRTL_RX_PASSIVE)   item("RX_PASSIVE");
        if (c & CAN_ERR_CRTL_TX_PASSIVE)   item("TX_PASSIVE");
        if (c & CAN_ERR_CRTL_ACTIVE)       item("ACTIVE");
        if (inner) item("UNSPEC");
        oss << "]";
    }

    if (err_id & CAN_ERR_PROT) {
        tag(), oss << "PROT[";
        bool inner = true;
        auto item = [&](const char* s) {
            if (!inner) oss << ' ';
            oss << s;
            inner = false;
        };
        uint8_t t = frame.data[2];
        if (t & CAN_ERR_PROT_BIT)         item("BIT");
        if (t & CAN_ERR_PROT_FORM)        item("FORM");
        if (t & CAN_ERR_PROT_STUFF)       item("STUFF");
        if (t & CAN_ERR_PROT_BIT0)        item("BIT0");
        if (t & CAN_ERR_PROT_BIT1)        item("BIT1");
        if (t & CAN_ERR_PROT_OVERLOAD)    item("OVERLOAD");
        if (t & CAN_ERR_PROT_ACTIVE)      item("ERR_ACTIVE");
        if (t & CAN_ERR_PROT_TX)          item("TX");
        if (inner) item("UNSPEC");
        oss << "]";
        tag(), oss << "loc=0x" << std::hex << (int)frame.data[3];
    }

    if (err_id & CAN_ERR_TRX) {
        tag(), oss << "TRX[";
        bool inner = true;
        auto item = [&](const char* s) {
            if (!inner) oss << ' ';
            oss << s;
            inner = false;
        };
        uint8_t t = frame.data[4];
        switch (t & 0x0F) {
            case CAN_ERR_TRX_CANH_NO_WIRE:      item("CANH_NO_WIRE"); break;
            case CAN_ERR_TRX_CANH_SHORT_TO_BAT: item("CANH_SHORT_TO_BAT"); break;
            case CAN_ERR_TRX_CANH_SHORT_TO_VCC: item("CANH_SHORT_TO_VCC"); break;
            case CAN_ERR_TRX_CANH_SHORT_TO_GND: item("CANH_SHORT_TO_GND"); break;
            case 0x00: break;
            default: oss << "CANH_" << (int)(t & 0x0F); inner = false; break;
        }
        switch (t & 0xF0) {
            case CAN_ERR_TRX_CANL_NO_WIRE:      item("CANL_NO_WIRE"); break;
            case CAN_ERR_TRX_CANL_SHORT_TO_BAT: item("CANL_SHORT_TO_BAT"); break;
            case CAN_ERR_TRX_CANL_SHORT_TO_VCC: item("CANL_SHORT_TO_VCC"); break;
            case CAN_ERR_TRX_CANL_SHORT_TO_GND: item("CANL_SHORT_TO_GND"); break;
            case CAN_ERR_TRX_CANL_SHORT_TO_CANH:item("CANL_SHORT_TO_CANH"); break;
            case 0x00: break;
            default: oss << "CANL_" << (int)((t & 0xF0) >> 4); inner = false; break;
        }
        if (inner) item("UNSPEC");
        oss << "]";
    }

    if (err_id & CAN_ERR_ACK)       { tag(); oss << "ACK"; }
    if (err_id & CAN_ERR_BUSOFF)    { tag(); oss << "BUSOFF"; }
    if (err_id & CAN_ERR_BUSERROR)  { tag(); oss << "BUSERROR"; }
    if (err_id & CAN_ERR_RESTARTED) { tag(); oss << "RESTARTED"; }

    oss << " tx_err=" << (int)frame.data[6]
        << " rx_err=" << (int)frame.data[7];

    return oss.str();
}

} // namespace can_utils

#endif // CAN_ERROR_FORMATTER_HPP
