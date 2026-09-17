#include "remote_control.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint16_t crc16(const char *text, size_t length)
{
    uint16_t crc = 0xffffU;
    for (size_t index = 0U; index < length; ++index) {
        crc ^= (uint16_t)(uint8_t)text[index] << 8U;
        for (unsigned int bit = 0U; bit < 8U; ++bit) {
            crc = (crc & 0x8000U) != 0U
                      ? (uint16_t)((crc << 1U) ^ 0x1021U)
                      : (uint16_t)(crc << 1U);
        }
    }
    return crc;
}

int main(void)
{
    char frame[64] = {0};
    const int body_length =
        snprintf(frame, sizeof(frame), "P,1,2,1600,200,1800");
    const uint16_t crc = crc16(frame, (size_t)body_length);
    const int frame_length = snprintf(frame + body_length,
                                      sizeof(frame) - (size_t)body_length,
                                      "*%04X\r\n", crc) + body_length;
    uint8_t slot[REMOTE_CONTROL_SLOT_BYTES] = {0};
    memcpy(slot, frame, (size_t)frame_length);

    RemoteControl_Init();
    assert(RemoteControl_SubmitSlot(slot, sizeof(slot)) ==
           REMOTE_CONTROL_SLOT_COMMAND);
    RemoteControlCommand command = {0};
    assert(RemoteControl_TryRead(&command));
    assert(command.pid_update);
    assert(command.values_in_range);
    assert(command.session == 1U && command.sequence == 2U);
    assert(command.pid_kp_tenths == 1600U);
    assert(command.pid_ki_hundredths == 200U);
    assert(command.pid_kd_tenths == 1800U);
    memset(frame, 0, sizeof(frame));
    const int control_body_length =
        snprintf(frame, sizeof(frame), "C,1,3,1000,0,1,50,0,20,-5,0,0");
    const uint16_t control_crc =
        crc16(frame, (size_t)control_body_length);
    const int control_frame_length =
        snprintf(frame + control_body_length,
                 sizeof(frame) - (size_t)control_body_length,
                 "*%04X\r\n", control_crc) + control_body_length;
    memset(slot, 0, sizeof(slot));
    memcpy(slot, frame, (size_t)control_frame_length);
    assert(RemoteControl_SubmitSlot(slot, sizeof(slot)) ==
           REMOTE_CONTROL_SLOT_COMMAND);
    assert(RemoteControl_TryRead(&command));
    assert(!command.pid_update && command.values_in_range);
    assert(command.ambient_temperature_c == -5);
    assert(command.target_temperature_c == 50U);
    puts("remote control PID tests passed");
    return 0;
}
