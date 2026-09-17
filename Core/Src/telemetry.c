#include "telemetry.h"
#include "main.h"
#include "remote_control.h"

#include <string.h>

/*
 * STM32 -> ESP32 telemetry link (SPI mode 1, MSB first):
 *   PB13  SPI2_SCK
 *   PB14  active-low frame select (GPIO, deliberately not PB12/SPI2_NSS)
 *   PB15  half-duplex data (STM32 output for telemetry, input for commands)
 *
 * PB12 remains available to the rotary encoder.  The ESP32 treats each
 * low-going PB14 interval as one fixed-size transaction.  One telemetry
 * transaction is followed by a separate command transaction after PB15 has
 * been released.
 */
#define TELEMETRY_CS_PIN GPIO_PIN_14
#define TELEMETRY_SPI_TIMEOUT_MS 10U
#define TELEMETRY_SPI_POLL_BUDGET 200000U
#define TELEMETRY_FRAME_BYTES 64U
#define TELEMETRY_COMMAND_TURNAROUND_MS 20U
#define TELEMETRY_COMMAND_RELEASE_MS 1U
#define TELEMETRY_STARTUP_IDLE_MS 250U
#define TELEMETRY_SYNC_PROBE_COUNT 2U
#define TELEMETRY_SPI_PRESCALER 128U
_Static_assert(TELEMETRY_FRAME_BYTES == 64U,
               "SPI telemetry transaction must remain 64 bytes");
_Static_assert(REMOTE_CONTROL_SLOT_BYTES == 64U,
               "SPI command transaction must remain 64 bytes");

/* Includes control state and command acknowledgement fields. */
static uint8_t tx_buffer[TELEMETRY_FRAME_BYTES];
static uint8_t command_buffer[REMOTE_CONTROL_SLOT_BYTES];
static uint32_t last_byte_arm_cycles;
static bool link_synchronized;

#define TELEMETRY_SPI_CR1_BASE                                            \
  (SPI_CR1_MSTR | SPI_CR1_CPHA | SPI_CR1_BR_2 | SPI_CR1_BR_1 |          \
   SPI_CR1_SSM | SPI_CR1_SSI)

static void ConfigureDataPinForReceive(void)
{
  GPIO_InitTypeDef gpio = {0};
  gpio.Pin = GPIO_PIN_15;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &gpio);
}

static void ConfigureDataPinForTransmit(void)
{
  GPIO_InitTypeDef gpio = {0};
  gpio.Pin = GPIO_PIN_15;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &gpio);
}

static void ConfigureSpi(bool transmit)
{
  __HAL_RCC_SPI2_FORCE_RESET();
  __HAL_RCC_SPI2_RELEASE_RESET();

  /* APB1 is 36 MHz. /128 gives a conservative 281.25 kbit/s link and
     provides a wide, deterministic window for stopping one-line receive. */
  /* PB15 is the regular SPI2 MOSI output in the request phase.  Classic
     STM32F1 can emit an idle-zero stream after the first byte when
     BIDIMODE+BIDIOE is repeatedly stopped and restarted; using the normal
     MOSI datapath avoids that erratum-like behaviour.  PB14 remains a GPIO
     chip select, so the unused full-duplex MISO input has no pin ownership.
     The reply phase switches to one-line receive because the ESP32 returns
     data over the same physical PB15 wire. */
  SPI2->CR1 = TELEMETRY_SPI_CR1_BASE |
              (transmit ? 0U : SPI_CR1_BIDIMODE);
  SPI2->CR2 = 0U;

  if (transmit)
  {
    ConfigureDataPinForTransmit();
  }
  else
  {
    ConfigureDataPinForReceive();
  }
}

static void ReleaseDataLine(void)
{
  CLEAR_BIT(SPI2->CR1, SPI_CR1_SPE);
  CLEAR_BIT(SPI2->CR1, SPI_CR1_BIDIOE);
  ConfigureDataPinForReceive();
}

static void EnableCycleCounter(void)
{
  SET_BIT(CoreDebug->DEMCR, CoreDebug_DEMCR_TRCENA_Msk);
  DWT->CYCCNT = 0U;
  SET_BIT(DWT->CTRL, DWT_CTRL_CYCCNTENA_Msk);

  const uint32_t spi_clock_hz =
      HAL_RCC_GetPCLK1Freq() / TELEMETRY_SPI_PRESCALER;
  last_byte_arm_cycles =
      (HAL_RCC_GetHCLKFreq() + spi_clock_hz - 1U) / spi_clock_hz;
}

static void WaitCoreCycles(uint32_t cycles)
{
  const uint32_t started = DWT->CYCCNT;
  while ((uint32_t)(DWT->CYCCNT - started) < cycles)
  {
  }
}

static RemoteControlSlotResult ReceiveCommandSlot(void);
static RemoteControlSlotResult SynchronizeLink(void);

void Telemetry_Init(void)
{
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_SPI2_CLK_ENABLE();

  /* Deassert frame select before changing PB14 to an output. */
  HAL_GPIO_WritePin(GPIOB, TELEMETRY_CS_PIN, GPIO_PIN_SET);

  GPIO_InitTypeDef gpio = {0};
  gpio.Pin = GPIO_PIN_13;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &gpio);

  gpio.Pin = TELEMETRY_CS_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &gpio);

  /* A pulled-up input distinguishes an actively driven reply from a released
     or disconnected PB15. Never drive the shared wire until an RX-only probe
     has consumed the ESP32's reply state and received a CRC-valid NOP/command. */
  ConfigureSpi(false);
  EnableCycleCounter();
  link_synchronized = false;
  HAL_Delay(TELEMETRY_STARTUP_IDLE_MS);
  (void)SynchronizeLink();

  /* A reply that was already armed before an STM32 reset can contain the
     previous controller instance's RUN heartbeat. Never apply a command
     discovered during boot synchronization. After the first full telemetry
     frame, the ESP32 observes the reset tick and either cancels that RUN or
     retransmits a still-pending stopped-state request. */
  RemoteControlCommand discarded_startup_command;
  while (RemoteControl_TryRead(&discarded_startup_command))
  {
  }
}

static void WriteU16Le(uint8_t *out, uint16_t value)
{
  out[0] = (uint8_t)value;
  out[1] = (uint8_t)(value >> 8U);
}

static void WriteU32Le(uint8_t *out, uint32_t value)
{
  out[0] = (uint8_t)value;
  out[1] = (uint8_t)(value >> 8U);
  out[2] = (uint8_t)(value >> 16U);
  out[3] = (uint8_t)(value >> 24U);
}

static uint16_t TelemetryCrc16(const uint8_t *data, size_t length)
{
  uint16_t crc = UINT16_C(0xFFFF);
  for (size_t index = 0U; index < length; ++index)
  {
    crc ^= (uint16_t)data[index] << 8U;
    for (uint32_t bit = 0U; bit < 8U; ++bit)
    {
      crc = (crc & UINT16_C(0x8000)) != 0U
                ? (uint16_t)((crc << 1U) ^ UINT16_C(0x1021))
                : (uint16_t)(crc << 1U);
    }
  }
  return crc;
}

static bool WaitForSpiFlag(uint32_t flag, bool asserted, uint32_t deadline,
                           uint32_t *budget)
{
  while (((SPI2->SR & flag) != 0U) != asserted)
  {
    if (*budget == 0U ||
        (int32_t)(HAL_GetTick() - deadline) >= 0)
    {
      return false;
    }
    --*budget;
  }
  return true;
}

static bool TransmitFrame(const uint8_t *data)
{
  uint32_t budget = TELEMETRY_SPI_POLL_BUDGET;
  const uint32_t deadline = HAL_GetTick() + TELEMETRY_SPI_TIMEOUT_MS;
  bool sent = true;

  ConfigureSpi(true);
  SET_BIT(SPI2->CR1, SPI_CR1_SPE);
  HAL_GPIO_WritePin(GPIOB, TELEMETRY_CS_PIN, GPIO_PIN_RESET);
  for (uint32_t index = 0U; index < TELEMETRY_FRAME_BYTES; ++index)
  {
    if (!WaitForSpiFlag(SPI_SR_TXE, true, deadline, &budget))
    {
      sent = false;
      break;
    }

    *(__IO uint8_t *)&SPI2->DR = (uint8_t)data[index];
  }

  if (sent)
  {
    sent = WaitForSpiFlag(SPI_SR_TXE, true, deadline, &budget) &&
           WaitForSpiFlag(SPI_SR_BSY, false, deadline, &budget);
  }

  /* PB15 must be high impedance before the ESP32 can select its reply driver.
     Keep CS low until after SPE and BIDIOE have been cleared. */
  ReleaseDataLine();
  HAL_GPIO_WritePin(GPIOB, TELEMETRY_CS_PIN, GPIO_PIN_SET);

  if (!sent)
  {
    RemoteControl_RecordTransportError();
  }
  return sent;
}

static RemoteControlSlotResult ReceiveCommandSlot(void)
{
  uint32_t budget = TELEMETRY_SPI_POLL_BUDGET;
  const uint32_t deadline = HAL_GetTick() + TELEMETRY_SPI_TIMEOUT_MS;
  bool received = true;
  uint32_t interrupt_state = 0U;
  bool interrupts_masked = false;

  memset(command_buffer, UINT8_MAX, sizeof(command_buffer));
  ConfigureSpi(false);
  HAL_GPIO_WritePin(GPIOB, TELEMETRY_CS_PIN, GPIO_PIN_RESET);
  SET_BIT(SPI2->CR1, SPI_CR1_SPE);

  /* In one-line master receive mode, enabling SPI starts continuous clocks.
     Read the first 61 bytes normally. The final three-byte section runs with
     interrupts masked so SPE can be cleared in the reference-manual window. */
  for (uint32_t index = 0U; index < REMOTE_CONTROL_SLOT_BYTES - 3U; ++index)
  {
    if (!WaitForSpiFlag(SPI_SR_RXNE, true, deadline, &budget))
    {
      received = false;
      break;
    }
    command_buffer[index] = *(__IO uint8_t *)&SPI2->DR;
  }

  if (received)
  {
    interrupt_state = __get_PRIMASK();
    __disable_irq();
    interrupts_masked = true;

    for (uint32_t index = REMOTE_CONTROL_SLOT_BYTES - 3U;
         index < REMOTE_CONTROL_SLOT_BYTES - 1U; ++index)
    {
      if (!WaitForSpiFlag(SPI_SR_RXNE, true, deadline, &budget))
      {
        received = false;
        break;
      }
      command_buffer[index] = *(__IO uint8_t *)&SPI2->DR;
    }
  }

  if (received)
  {
    /* RM0008: after the second-to-last RXNE, wait one SCK period, clear SPE
       during the last frame, then wait for/read the final RXNE. At /128 one
       SCK is 256 CPU cycles, leaving ample margin before the final bit. */
    WaitCoreCycles(last_byte_arm_cycles);
    CLEAR_BIT(SPI2->CR1, SPI_CR1_SPE);
    __DSB();
    if (WaitForSpiFlag(SPI_SR_RXNE, true, deadline, &budget))
    {
      command_buffer[REMOTE_CONTROL_SLOT_BYTES - 1U] =
          *(__IO uint8_t *)&SPI2->DR;
    }
    else
    {
      received = false;
    }
  }
  else
  {
    CLEAR_BIT(SPI2->CR1, SPI_CR1_SPE);
  }

  if (interrupts_masked && interrupt_state == 0U)
  {
    __enable_irq();
  }
  ReleaseDataLine();
  HAL_GPIO_WritePin(GPIOB, TELEMETRY_CS_PIN, GPIO_PIN_SET);

  /* The ESP32 disables its GPIO23 output in its transaction callback. Leave
     PB15 as an input between samples, including this guard interval. */
  HAL_Delay(TELEMETRY_COMMAND_RELEASE_MS);
  if (!received)
  {
    RemoteControl_RecordTransportError();
    return REMOTE_CONTROL_SLOT_INVALID;
  }
  return RemoteControl_SubmitSlot(command_buffer, sizeof(command_buffer));
}

static RemoteControlSlotResult SynchronizeLink(void)
{
  for (uint32_t attempt = 0U; attempt < TELEMETRY_SYNC_PROBE_COUNT; ++attempt)
  {
    const RemoteControlSlotResult result = ReceiveCommandSlot();
    if (result != REMOTE_CONTROL_SLOT_INVALID)
    {
      link_synchronized = true;
      /* Give the ESP32 task time to disable GPIO23 and arm its 64-byte RX. */
      HAL_Delay(TELEMETRY_COMMAND_TURNAROUND_MS);
      return result;
    }
    if (attempt + 1U < TELEMETRY_SYNC_PROBE_COUNT)
    {
      /* If this probe reached the ESP32's RX state, it replies with a NOP on
         the next probe even though the 64-byte pseudo-telemetry was invalid. */
      HAL_Delay(TELEMETRY_COMMAND_TURNAROUND_MS);
    }
  }

  link_synchronized = false;
  RemoteControl_RecordTransportError();
  return REMOTE_CONTROL_SLOT_INVALID;
}

bool Telemetry_Send(uint32_t tick_ms, int32_t temperature_centi_c,
                    uint32_t humidity_milli_percent,
                    bool valid, uint8_t state_bits,
                    uint16_t output_permille, uint8_t target_temperature_c,
                    uint8_t target_humidity_percent, uint8_t fault,
                    int8_t ambient_temperature_c,
                    uint32_t ack_session, uint32_t ack_sequence,
                    uint8_t ack_result, uint32_t rx_errors)
{
  if (!link_synchronized)
  {
    const RemoteControlSlotResult sync_result = SynchronizeLink();
    if (sync_result == REMOTE_CONTROL_SLOT_INVALID)
    {
      return false;
    }
    if (sync_result == REMOTE_CONTROL_SLOT_COMMAND)
    {
      /* Let the main loop consume the recovered command before another reply
         slot can attempt to enqueue a second command. */
      return true;
    }
  }

  memset(tx_buffer, 0, sizeof(tx_buffer));
  tx_buffer[0] = (uint8_t)'T';
  tx_buffer[1] = (uint8_t)'G';
  tx_buffer[2] = 3U;
  tx_buffer[3] = 40U;
  WriteU32Le(&tx_buffer[4], tick_ms);
  WriteU32Le(&tx_buffer[8], (uint32_t)temperature_centi_c);
  tx_buffer[12] = (uint8_t)(ambient_temperature_c + 10);
  /* Bytes 13..15 are reserved zero in v3. */
  WriteU32Le(&tx_buffer[16], humidity_milli_percent);
  tx_buffer[20] = valid ? 1U : 0U;
  tx_buffer[21] = state_bits;
  WriteU16Le(&tx_buffer[22], output_permille);
  tx_buffer[24] = target_temperature_c;
  tx_buffer[25] = target_humidity_percent;
  tx_buffer[26] = fault;
  tx_buffer[27] = ack_result;
  WriteU32Le(&tx_buffer[28], ack_session);
  WriteU32Le(&tx_buffer[32], ack_sequence);
  WriteU32Le(&tx_buffer[36], rx_errors);
  WriteU16Le(&tx_buffer[62], TelemetryCrc16(tx_buffer, 62U));

  if (!TransmitFrame(tx_buffer))
  {
    link_synchronized = false;
    return false;
  }

  /* ESP32 parses the 64-byte telemetry transaction and arms its reply while
     CS is high and PB15 is released. */
  HAL_Delay(TELEMETRY_COMMAND_TURNAROUND_MS);
  const RemoteControlSlotResult result = ReceiveCommandSlot();
  if (result == REMOTE_CONTROL_SLOT_INVALID)
  {
    link_synchronized = false;
    return false;
  }
  return true;
}
