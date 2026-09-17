#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  uint32_t session;
  uint32_t sequence;
  uint32_t seen_tick_ms;
  bool run;
  bool temperature_enabled;
  uint8_t target_temperature_c;
  bool humidity_enabled;
  uint8_t target_humidity_percent;
  int8_t ambient_temperature_c;
  bool clear_fault;
  bool persist;
  bool pid_update;
  uint16_t pid_kp_tenths;
  uint16_t pid_ki_hundredths;
  uint16_t pid_kd_tenths;
  bool values_in_range;
} RemoteControlCommand;

#define REMOTE_PID_KP_MAX_TENTHS 5000U
#define REMOTE_PID_KI_MAX_HUNDREDTHS 5000U
#define REMOTE_PID_KD_MAX_TENTHS 10000U
#define REMOTE_MIN_AMBIENT_TEMPERATURE_C (-10)
#define REMOTE_MAX_AMBIENT_TEMPERATURE_C 50
#define REMOTE_DEFAULT_AMBIENT_TEMPERATURE_C 22

#define REMOTE_CONTROL_SLOT_BYTES 64U

typedef enum
{
  REMOTE_CONTROL_SLOT_INVALID = 0,
  REMOTE_CONTROL_SLOT_NOP = 1,
  REMOTE_CONTROL_SLOT_COMMAND = 2,
} RemoteControlSlotResult;

/* ESP32 command input arrives in a fixed-size PB15 half-duplex SPI slot. */
void RemoteControl_Init(void);
RemoteControlSlotResult RemoteControl_SubmitSlot(const uint8_t *slot,
                                                size_t length);
void RemoteControl_RecordTransportError(void);
bool RemoteControl_TryRead(RemoteControlCommand *command);
uint32_t RemoteControl_GetRxErrorCount(void);

#ifdef __cplusplus
}
#endif

#endif /* REMOTE_CONTROL_H */
