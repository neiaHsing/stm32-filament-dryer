#include "main.h"

#include "aht20.h"
#include "bmp280.h"
#include "driver_oled.h"
#include "gpio.h"
#include "i2c.h"
#include "settings_storage.h"
#include "spi.h"

#include <cstring>

namespace
{
constexpr uint32_t kLoopDelayMs = 2U;
constexpr uint32_t kButtonDebounceMs = 25U;
constexpr uint32_t kButtonLongPressMs = 1000U;
constexpr uint32_t kSensorPeriodMs = 1000U;
constexpr uint32_t kSensorReconnectPeriodMs = 2000U;
constexpr uint32_t kAht20ConversionMs = 85U;
constexpr uint32_t kDisplayPeriodMs = 1000U;
constexpr uint32_t kFanRunConfirmMs = 200U;
constexpr uint32_t kFanStopConfirmMs = 500U;
constexpr uint32_t kFanStartupGraceMs = 6000U;
constexpr int32_t kAbsoluteOvertemperatureCentiC = 7500;
constexpr int32_t kOvertemperatureResetCentiC = 7000;
constexpr int32_t kNoTargetSafetyLimitCentiC = 7000;
constexpr int32_t kTemperatureHysteresisCentiC = 100;
constexpr uint32_t kHumidityHysteresisMilliPercent = 2000U;
constexpr int16_t kTargetDisabled = -1;

constexpr uint8_t kDefaultTargetTemperatureC = 55U;
constexpr uint8_t kMinimumTargetTemperatureC = 30U;
constexpr uint8_t kMaximumTargetTemperatureC = 70U;
constexpr uint8_t kDefaultTargetHumidityPercent = 20U;
constexpr uint8_t kMinimumTargetHumidityPercent = 10U;
constexpr uint8_t kMaximumTargetHumidityPercent = 80U;

constexpr uint32_t kPwmTimerClockHz = 72000000U;
constexpr uint32_t kPwmCounterClockHz = 10000U;
constexpr uint32_t kPwmFrequencyHz = 10U;
constexpr bool kXyMosActiveHigh = true;
constexpr uint32_t kMotorPwmFrequencyHz = 20000U;
constexpr uint32_t kMotorPwmPeriodCounts =
    kPwmTimerClockHz / kMotorPwmFrequencyHz;
constexpr uint32_t kMotorDutyPercent = 100U;
static_assert(kMotorDutyPercent <= 100U, "Motor duty must be 0..100 percent");
volatile int8_t g_encoderTransitionAccumulator = 0;
volatile int16_t g_encoderPendingSteps = 0;
volatile uint8_t g_encoderPreviousState = 3U;
bool g_displayCacheValid = false;
char g_displayCache[4][17] = {};

enum class UiPage : uint8_t
{
  Dashboard,
  SetTemperature,
  SetHumidity,
  StartConfirm,
};

enum class ButtonEvent : uint8_t
{
  None,
  ShortPress,
  LongPress,
};

enum class FaultCode : uint8_t
{
  None,
  Fan,
  Sensor,
  Overtemperature,
};

struct Settings
{
  uint8_t targetTemperatureC = kDefaultTargetTemperatureC;
  uint8_t targetHumidityPercent = kDefaultTargetHumidityPercent;
  bool temperatureEnabled = true;
  bool humidityEnabled = true;
};

struct SensorData
{
  int32_t ahtTemperatureCentiC = 0;
  int32_t bmpTemperatureCentiC = 0;
  uint32_t humidityMilliPercent = 0U;
  bool valid = false;
};

struct FanState
{
  bool rawRunning = false;
  bool running = false;
  bool sampleValid = false;
  uint32_t rawChangedTick = 0U;
};

struct RuntimeState
{
  bool running = false;
  bool heaterOn = false;
  bool motorOn = false;
  bool dryingNeeded = true;
  FaultCode fault = FaultCode::None;
  uint32_t runStartTick = 0U;
  uint32_t elapsedSeconds = 0U;
};

struct ButtonState
{
  bool rawPressed = false;
  bool stablePressed = false;
  bool longPressReported = false;
  uint32_t rawChangedTick = 0U;
  uint32_t pressStartedTick = 0U;
};

bool TimeReached(uint32_t now, uint32_t deadline)
{
  return static_cast<int32_t>(now - deadline) >= 0;
}

bool HeatingConfigured(const Settings &settings)
{
  return settings.temperatureEnabled || settings.humidityEnabled;
}

void AdjustOptionalTarget(int16_t &value, int8_t step, int16_t minimum,
                          int16_t maximum)
{
  if (step > 0)
  {
    if (value == kTargetDisabled)
    {
      value = minimum;
    }
    else if (value < maximum)
    {
      ++value;
    }
  }
  else if (step < 0 && value != kTargetDisabled)
  {
    if (value <= minimum)
    {
      value = kTargetDisabled;
    }
    else
    {
      --value;
    }
  }
}

bool InitializeSensorStack(bool recoverBus)
{
  if (recoverBus && I2C2_Recover() != HAL_OK)
  {
    return false;
  }

  const bool aht20Ready = AHT20_Init(&hi2c2);
  const bool bmp280Ready = BMP280_Init(&hi2c2);
  return aht20Ready && bmp280Ready;
}

bool EncoderKeyPressed()
{
  return HAL_GPIO_ReadPin(ENCODER_KEY_GPIO_Port, ENCODER_KEY_Pin) ==
         GPIO_PIN_RESET;
}

uint8_t ReadEncoderState()
{
  const uint8_t s1 = HAL_GPIO_ReadPin(ENCODER_S1_GPIO_Port,
                                      ENCODER_S1_Pin) == GPIO_PIN_SET;
  const uint8_t s2 = HAL_GPIO_ReadPin(ENCODER_S2_GPIO_Port,
                                      ENCODER_S2_Pin) == GPIO_PIN_SET;
  return static_cast<uint8_t>((s1 << 1U) | s2);
}

void CaptureEncoderTransition()
{
  static constexpr int8_t transitionTable[16] = {
      0, -1, 1, 0,
      1, 0, 0, -1,
      -1, 0, 0, 1,
      0, 1, -1, 0,
  };

  const uint8_t currentState = ReadEncoderState();
  g_encoderTransitionAccumulator +=
      transitionTable[(g_encoderPreviousState << 2U) | currentState];
  g_encoderPreviousState = currentState;

  if (g_encoderTransitionAccumulator >= 4)
  {
    g_encoderTransitionAccumulator = 0;
    if (g_encoderPendingSteps < 1000)
    {
      ++g_encoderPendingSteps;
    }
  }
  else if (g_encoderTransitionAccumulator <= -4)
  {
    g_encoderTransitionAccumulator = 0;
    if (g_encoderPendingSteps > -1000)
    {
      --g_encoderPendingSteps;
    }
  }
}

int8_t ReadEncoderStep()
{
  const uint32_t interruptState = __get_PRIMASK();
  __disable_irq();
  int8_t step = 0;
  if (g_encoderPendingSteps > 0)
  {
    --g_encoderPendingSteps;
    step = 1;
  }
  else if (g_encoderPendingSteps < 0)
  {
    ++g_encoderPendingSteps;
    step = -1;
  }
  if (interruptState == 0U)
  {
    __enable_irq();
  }
  return step;
}

ButtonEvent UpdateButton(ButtonState &button, uint32_t now)
{
  const bool pressed = EncoderKeyPressed();
  if (pressed != button.rawPressed)
  {
    button.rawPressed = pressed;
    button.rawChangedTick = now;
  }

  if (button.stablePressed != button.rawPressed &&
      now - button.rawChangedTick >= kButtonDebounceMs)
  {
    button.stablePressed = button.rawPressed;
    if (button.stablePressed)
    {
      button.pressStartedTick = now;
      button.longPressReported = false;
    }
    else if (!button.longPressReported)
    {
      return ButtonEvent::ShortPress;
    }
  }

  if (button.stablePressed && !button.longPressReported &&
      now - button.pressStartedTick >= kButtonLongPressMs)
  {
    button.longPressReported = true;
    return ButtonEvent::LongPress;
  }
  return ButtonEvent::None;
}

void InitHeaterPwm()
{
  RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
  TIM4->CR1 = 0U;
  TIM4->PSC = (kPwmTimerClockHz / kPwmCounterClockHz) - 1U;
  TIM4->ARR = (kPwmCounterClockHz / kPwmFrequencyHz) - 1U;
  TIM4->CCR3 = 0U;
  TIM4->CCMR2 = (6U << TIM_CCMR2_OC3M_Pos) | TIM_CCMR2_OC3PE;
  TIM4->CCER = TIM_CCER_CC3E |
               (kXyMosActiveHigh ? 0U : TIM_CCER_CC3P);
  TIM4->EGR = TIM_EGR_UG;
  TIM4->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
}

void InitMotorPwm()
{
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
  TIM2->CR1 = 0U;
  TIM2->CCER = 0U;
  TIM2->PSC = 0U;
  TIM2->ARR = kMotorPwmPeriodCounts - 1U;
  TIM2->CCR1 = 0U;
  TIM2->CCMR1 = (6U << TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;
  TIM2->EGR = TIM_EGR_UG;
  TIM2->CCER = TIM_CCER_CC1E;
  TIM2->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
}

bool SetHeaterOutput(RuntimeState &runtime, bool enabled)
{
  if (runtime.heaterOn == enabled)
  {
    return false;
  }

  runtime.heaterOn = enabled;
  TIM4->CCR3 = enabled ? TIM4->ARR + 1U : 0U;
  TIM4->EGR = TIM_EGR_UG;
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin,
                    enabled ? GPIO_PIN_RESET : GPIO_PIN_SET);
  return true;
}

bool SetMotorOutput(RuntimeState &runtime, bool enabled)
{
  if (runtime.motorOn == enabled)
  {
    return false;
  }

  if (enabled)
  {
    HAL_GPIO_WritePin(MOTOR_IN2_GPIO_Port, MOTOR_IN2_Pin, GPIO_PIN_RESET);
    TIM2->CCR1 =
        (kMotorPwmPeriodCounts * kMotorDutyPercent) / 100U;
  }
  else
  {
    TIM2->CCR1 = 0U;
    HAL_GPIO_WritePin(MOTOR_IN2_GPIO_Port, MOTOR_IN2_Pin, GPIO_PIN_RESET);
  }
  TIM2->EGR = TIM_EGR_UG;
  runtime.motorOn = enabled;
  return true;
}

bool SetDryingOutputs(RuntimeState &runtime, bool enabled)
{
  bool changed = false;
  if (enabled)
  {
    changed = SetMotorOutput(runtime, true) || changed;
    changed = SetHeaterOutput(runtime, true) || changed;
  }
  else
  {
    changed = SetHeaterOutput(runtime, false) || changed;
    changed = SetMotorOutput(runtime, false) || changed;
  }
  return changed;
}

void UpdateElapsedTime(RuntimeState &runtime, uint32_t now)
{
  if (runtime.running)
  {
    runtime.elapsedSeconds = (now - runtime.runStartTick) / 1000U;
  }
}

bool StopRun(RuntimeState &runtime, uint32_t now)
{
  UpdateElapsedTime(runtime, now);
  const bool changed = runtime.running || runtime.heaterOn || runtime.motorOn;
  runtime.running = false;
  SetDryingOutputs(runtime, false);
  return changed;
}

bool LatchFault(RuntimeState &runtime, FaultCode fault, uint32_t now)
{
  const bool changed = runtime.fault != fault || runtime.running ||
                       runtime.heaterOn || runtime.motorOn;
  StopRun(runtime, now);
  runtime.fault = fault;
  return changed;
}

bool FanSignalIndicatesRunning()
{
  return HAL_GPIO_ReadPin(FAN_TACH_GPIO_Port, FAN_TACH_Pin) == GPIO_PIN_RESET;
}

bool UpdateFanState(FanState &fan, uint32_t now)
{
  const bool rawRunning = FanSignalIndicatesRunning();
  if (rawRunning != fan.rawRunning)
  {
    fan.rawRunning = rawRunning;
    fan.rawChangedTick = now;
  }

  const uint32_t confirmTime =
      fan.rawRunning ? kFanRunConfirmMs : kFanStopConfirmMs;
  if ((!fan.sampleValid || fan.running != fan.rawRunning) &&
      now - fan.rawChangedTick >= confirmTime)
  {
    fan.running = fan.rawRunning;
    fan.sampleValid = true;
    return true;
  }
  return false;
}

int32_t MaximumTemperature(const SensorData &sensor)
{
  return sensor.ahtTemperatureCentiC > sensor.bmpTemperatureCentiC
             ? sensor.ahtTemperatureCentiC
             : sensor.bmpTemperatureCentiC;
}

bool WorkAppearsNeeded(const Settings &settings, const SensorData &sensor)
{
  if (!HeatingConfigured(settings))
  {
    return true;
  }
  if (!sensor.valid)
  {
    return false;
  }
  if (settings.humidityEnabled)
  {
    const uint32_t target =
        static_cast<uint32_t>(settings.targetHumidityPercent) * 1000U;
    return sensor.humidityMilliPercent > target;
  }
  return MaximumTemperature(sensor) <
         static_cast<int32_t>(settings.targetTemperatureC) * 100;
}

bool StartRun(RuntimeState &runtime, const Settings &settings,
              const SensorData &sensor, const FanState &fan, uint32_t now)
{
  if (runtime.fault != FaultCode::None)
  {
    return false;
  }
  if (!fan.sampleValid || !fan.running)
  {
    runtime.fault = FaultCode::Fan;
    return false;
  }
  if (HeatingConfigured(settings))
  {
    if (!sensor.valid)
    {
      runtime.fault = FaultCode::Sensor;
      return false;
    }
    if (MaximumTemperature(sensor) >= kAbsoluteOvertemperatureCentiC)
    {
      runtime.fault = FaultCode::Overtemperature;
      return false;
    }
  }

  runtime.running = true;
  runtime.heaterOn = false;
  runtime.dryingNeeded =
      settings.humidityEnabled
          ? sensor.humidityMilliPercent >
                static_cast<uint32_t>(settings.targetHumidityPercent) * 1000U
          : settings.temperatureEnabled;
  runtime.runStartTick = now;
  runtime.elapsedSeconds = 0U;
  return true;
}

bool SensorValuesPlausible(const AHT20_Measurement &aht,
                           int32_t bmpTemperatureCentiC)
{
  return aht.temperature_centi_c >= -4000 &&
         aht.temperature_centi_c <= 8500 &&
         bmpTemperatureCentiC >= -4000 &&
         bmpTemperatureCentiC <= 8500 &&
         aht.humidity_milli_percent <= 100000U;
}

bool CanClearFault(const RuntimeState &runtime, const Settings &settings,
                   const SensorData &sensor, const FanState &fan)
{
  if (runtime.fault == FaultCode::Fan)
  {
    return fan.sampleValid && fan.running;
  }
  if (runtime.fault == FaultCode::Sensor && !HeatingConfigured(settings))
  {
    return true;
  }
  return sensor.valid &&
         MaximumTemperature(sensor) < kOvertemperatureResetCentiC;
}

bool UpdateControl(RuntimeState &runtime, const Settings &settings,
                   const SensorData &sensor, const FanState &fan,
                   uint32_t now)
{
  bool changed = false;
  UpdateElapsedTime(runtime, now);

  if (!runtime.running || runtime.fault != FaultCode::None)
  {
    return SetDryingOutputs(runtime, false) || changed;
  }

  const uint32_t runTimeMs = now - runtime.runStartTick;
  if (runTimeMs < kFanStartupGraceMs)
  {
    return SetDryingOutputs(runtime, false) || changed;
  }

  if (!fan.sampleValid || !fan.running)
  {
    return LatchFault(runtime, FaultCode::Fan, now);
  }

  if (!HeatingConfigured(settings))
  {
    runtime.dryingNeeded = false;
    return SetDryingOutputs(runtime, false) || changed;
  }

  if (!sensor.valid)
  {
    return LatchFault(runtime, FaultCode::Sensor, now);
  }

  if (MaximumTemperature(sensor) >= kAbsoluteOvertemperatureCentiC)
  {
    return LatchFault(runtime, FaultCode::Overtemperature, now);
  }

  const bool previousDryingNeeded = runtime.dryingNeeded;
  if (settings.humidityEnabled)
  {
    const uint32_t humidityTarget =
        static_cast<uint32_t>(settings.targetHumidityPercent) * 1000U;
    if (sensor.humidityMilliPercent <= humidityTarget)
    {
      runtime.dryingNeeded = false;
    }
    else if (sensor.humidityMilliPercent >=
             humidityTarget + kHumidityHysteresisMilliPercent)
    {
      runtime.dryingNeeded = true;
    }
  }
  else
  {
    runtime.dryingNeeded = true;
  }
  changed = previousDryingNeeded != runtime.dryingNeeded;

  if (!runtime.dryingNeeded)
  {
    return SetDryingOutputs(runtime, false) || changed;
  }

  const int32_t targetTemperature =
      settings.temperatureEnabled
          ? static_cast<int32_t>(settings.targetTemperatureC) * 100
          : kNoTargetSafetyLimitCentiC;
  const int32_t controlTemperature = MaximumTemperature(sensor);
  if (runtime.heaterOn && controlTemperature >= targetTemperature)
  {
    changed = SetDryingOutputs(runtime, false) || changed;
  }
  else if (!runtime.heaterOn &&
           controlTemperature <=
               targetTemperature - kTemperatureHysteresisCentiC)
  {
    changed = SetDryingOutputs(runtime, true) || changed;
  }
  return changed;
}

void InitializeLine(char line[17])
{
  std::memset(line, ' ', 16U);
  line[16] = '\0';
}

void PutText(char line[17], uint8_t position, const char *text)
{
  while (*text != '\0' && position < 16U)
  {
    line[position++] = *text++;
  }
}

void PutUnsigned(char line[17], uint8_t position, uint32_t value,
                 uint8_t width, bool zeroPad = false)
{
  if (position >= 16U || width == 0U)
  {
    return;
  }
  if (position + width > 16U)
  {
    width = static_cast<uint8_t>(16U - position);
  }

  for (uint8_t index = 0U; index < width; ++index)
  {
    const uint8_t output =
        static_cast<uint8_t>(position + width - 1U - index);
    line[output] = static_cast<char>('0' + value % 10U);
    value /= 10U;
    if (value == 0U)
    {
      if (zeroPad)
      {
        while (++index < width)
        {
          line[position + width - 1U - index] = '0';
        }
      }
      break;
    }
  }
}

void PutOptionalTarget(char line[17], uint8_t position, bool enabled,
                       uint8_t value, char unit)
{
  if (!enabled)
  {
    PutText(line, position, "OFF");
    return;
  }
  PutUnsigned(line, position, value, 2U, true);
  line[position + 2U] = unit;
}

void PutTemperature(char line[17], uint8_t position, int32_t centiC)
{
  int32_t tenths = centiC >= 0 ? (centiC + 5) / 10
                               : (centiC - 5) / 10;
  const bool negative = tenths < 0;
  uint32_t magnitude = static_cast<uint32_t>(negative ? -tenths : tenths);
  if (magnitude > 999U)
  {
    magnitude = 999U;
  }

  line[position] = negative ? '-' : ' ';
  line[position + 1U] = magnitude >= 100U
                            ? static_cast<char>('0' + (magnitude / 100U) % 10U)
                            : ' ';
  line[position + 2U] = static_cast<char>('0' + (magnitude / 10U) % 10U);
  line[position + 3U] = '.';
  line[position + 4U] = static_cast<char>('0' + magnitude % 10U);
}

void RenderDashboard(char lines[4][17], const Settings &settings,
                     const SensorData &sensor, const FanState &fan,
                     const RuntimeState &runtime, uint32_t now)
{
  if (sensor.valid)
  {
    PutText(lines[0], 0U, "T:");
    PutTemperature(lines[0], 2U, sensor.ahtTemperatureCentiC);
    lines[0][7] = 'C';
    PutText(lines[0], 9U, "H:");
    PutUnsigned(lines[0], 11U,
                (sensor.humidityMilliPercent + 500U) / 1000U, 3U);
    lines[0][14] = '%';
  }
  else
  {
    PutText(lines[0], 0U, "T: --.-C H: --%");
  }

  PutText(lines[1], 0U, "FAN:");
  if (fan.sampleValid)
  {
    PutText(lines[1], 5U, fan.running ? "RUN" : "STOP");
  }
  else
  {
    PutText(lines[1], 5U, "WAIT");
  }
  PutText(lines[1], 10U, "M:");
  PutText(lines[1], 12U, runtime.motorOn ? "ON" : "OFF");

  const uint32_t elapsed = runtime.running
                               ? (now - runtime.runStartTick) / 1000U
                               : runtime.elapsedSeconds;
  const uint32_t hours = (elapsed / 3600U) % 100U;
  const uint32_t minutes = (elapsed / 60U) % 60U;
  const uint32_t seconds = elapsed % 60U;
  PutText(lines[2], 0U, "TIME:");
  PutUnsigned(lines[2], 6U, hours, 2U, true);
  lines[2][8] = ':';
  PutUnsigned(lines[2], 9U, minutes, 2U, true);
  lines[2][11] = ':';
  PutUnsigned(lines[2], 12U, seconds, 2U, true);

  PutOptionalTarget(lines[3], 0U, settings.temperatureEnabled,
                    settings.targetTemperatureC, 'C');
  lines[3][3] = '/';
  PutOptionalTarget(lines[3], 4U, settings.humidityEnabled,
                    settings.targetHumidityPercent, '%');

  const char *status = "IDLE OFF";
  if (runtime.fault == FaultCode::Fan)
  {
    status = "FAN ERR";
  }
  else if (runtime.fault == FaultCode::Sensor ||
           (!sensor.valid && HeatingConfigured(settings)))
  {
    status = "SENS ERR";
  }
  else if (runtime.fault == FaultCode::Overtemperature)
  {
    status = "OVERHEAT";
  }
  else if (runtime.running && now - runtime.runStartTick < kFanStartupGraceMs)
  {
    status = "FAN WAIT";
  }
  else if (runtime.heaterOn)
  {
    status = "HEATING";
  }
  else if (runtime.running && !HeatingConfigured(settings))
  {
    status = "FAN ONLY";
  }
  else if (runtime.running && !runtime.dryingNeeded)
  {
    status = "RH HOLD";
  }
  else if (runtime.running && !settings.temperatureEnabled)
  {
    status = "70C HOLD";
  }
  else if (runtime.running)
  {
    status = "T HOLD";
  }
  PutText(lines[3], 8U, status);
}

void RenderScreen(UiPage page, int16_t editValue, bool startSelected,
                  const Settings &settings, const Settings &draftSettings,
                  const SensorData &sensor, const FanState &fan,
                  const RuntimeState &runtime, uint32_t now)
{
  char lines[4][17];
  for (auto &line : lines)
  {
    InitializeLine(line);
  }

  if (page == UiPage::Dashboard)
  {
    RenderDashboard(lines, settings, sensor, fan, runtime, now);
  }
  else if (page == UiPage::SetTemperature)
  {
    PutText(lines[0], 0U, "SET TEMPERATURE");
    PutText(lines[1], 0U, "LIMIT:");
    if (editValue == kTargetDisabled)
    {
      PutText(lines[1], 8U, "OFF");
    }
    else
    {
      PutUnsigned(lines[1], 8U, static_cast<uint32_t>(editValue), 2U, true);
      lines[1][10] = 'C';
    }
    PutText(lines[2], 0U, "ROTATE TO SET");
    PutText(lines[3], 0U, "PRESS: NEXT");
  }
  else if (page == UiPage::SetHumidity)
  {
    PutText(lines[0], 0U, "SET HUMIDITY");
    PutText(lines[1], 0U, "TARGET:");
    if (editValue == kTargetDisabled)
    {
      PutText(lines[1], 8U, "OFF");
    }
    else
    {
      PutUnsigned(lines[1], 8U, static_cast<uint32_t>(editValue), 2U, true);
      lines[1][10] = '%';
    }
    PutText(lines[2], 0U, "ROTATE TO SET");
    PutText(lines[3], 0U, "PRESS: NEXT");
  }
  else
  {
    PutText(lines[0], 0U, "START TASK?");
    PutText(lines[1], 0U, "T:");
    PutOptionalTarget(lines[1], 2U, draftSettings.temperatureEnabled,
                      draftSettings.targetTemperatureC, 'C');
    PutText(lines[1], 6U, "H:");
    PutOptionalTarget(lines[1], 8U, draftSettings.humidityEnabled,
                      draftSettings.targetHumidityPercent, '%');
    PutText(lines[2], 0U, startSelected ? "> START" : "  START");
    PutText(lines[3], 0U,
            startSelected ? "  SAVE ONLY" : "> SAVE ONLY");
  }

  for (uint8_t row = 0U; row < 4U; ++row)
  {
    if (!g_displayCacheValid ||
        std::memcmp(g_displayCache[row], lines[row], sizeof(lines[row])) != 0)
    {
      OLED_PrintString(0U, static_cast<uint8_t>(row * 2U), lines[row]);
      if (!OLED_IsReady())
      {
        g_displayCacheValid = false;
        return;
      }
      std::memcpy(g_displayCache[row], lines[row], sizeof(lines[row]));
    }
  }
  g_displayCacheValid = true;
}
} // namespace

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == ENCODER_S1_Pin || GPIO_Pin == ENCODER_S2_Pin)
  {
    CaptureEncoderTransition();
  }
}

void SystemClock_Config(void);

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  g_encoderPreviousState = ReadEncoderState();
  g_encoderTransitionAccumulator = 0;
  g_encoderPendingSteps = 0;
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();

  InitHeaterPwm();
  InitMotorPwm();
  RuntimeState runtime;
  SetDryingOutputs(runtime, false);

  HAL_Delay(100U);
  bool displayReady = OLED_Begin();
  uint32_t nextOledProbeTick = HAL_GetTick() + 2000U;
  if (displayReady)
  {
    OLED_PrintString(0U, 0U, "DRYER STARTING");
  }

  Settings settings;
  const bool settingsStorageReady = SettingsStorage_Init(&hspi1);
  StoredSettings storedSettings{};
  if (settingsStorageReady && SettingsStorage_Load(&storedSettings) &&
      storedSettings.target_temperature_c >= kMinimumTargetTemperatureC &&
      storedSettings.target_temperature_c <= kMaximumTargetTemperatureC &&
      storedSettings.target_humidity_percent >= kMinimumTargetHumidityPercent &&
      storedSettings.target_humidity_percent <= kMaximumTargetHumidityPercent)
  {
    settings.targetTemperatureC = storedSettings.target_temperature_c;
    settings.targetHumidityPercent = storedSettings.target_humidity_percent;
    settings.temperatureEnabled = storedSettings.temperature_enabled;
    settings.humidityEnabled = storedSettings.humidity_enabled;
  }

  bool sensorStackReady = InitializeSensorStack(false);

  SensorData sensor;
  FanState fan;
  fan.rawRunning = FanSignalIndicatesRunning();
  fan.rawChangedTick = HAL_GetTick();

  UiPage page = UiPage::Dashboard;
  Settings draftSettings = settings;
  int16_t editValue = draftSettings.targetTemperatureC;
  bool startSelected = true;
  const uint32_t initialTick = HAL_GetTick();
  const bool keyPressed = EncoderKeyPressed();
  ButtonState button{keyPressed, keyPressed, false, initialTick, initialTick};

  bool aht20MeasurementPending = false;
  uint32_t aht20MeasurementStartedTick = 0U;
  uint32_t nextSensorTick = initialTick;
  uint32_t nextSensorReconnectTick =
      initialTick + kSensorReconnectPeriodMs;
  uint32_t nextDisplayTick = initialTick;
  bool displayDirty = true;

  while (1)
  {
    const uint32_t now = HAL_GetTick();

    if (!displayReady && TimeReached(now, nextOledProbeTick))
    {
      displayReady = OLED_Begin();
      nextOledProbeTick = HAL_GetTick() + 2000U;
      if (displayReady)
      {
        g_displayCacheValid = false;
        displayDirty = true;
      }
    }

    if (UpdateFanState(fan, now))
    {
      displayDirty = true;
    }
    if (runtime.fault == FaultCode::Fan && fan.sampleValid && fan.running)
    {
      runtime.fault = FaultCode::None;
      displayDirty = true;
    }

    if (!sensorStackReady &&
        TimeReached(now, nextSensorReconnectTick))
    {
      aht20MeasurementPending = false;
      sensorStackReady = InitializeSensorStack(true);
      const uint32_t recoveryFinishedTick = HAL_GetTick();
      if (sensorStackReady)
      {
        nextSensorTick = recoveryFinishedTick;
      }
      else
      {
        nextSensorReconnectTick =
            recoveryFinishedTick + kSensorReconnectPeriodMs;
      }
      displayDirty = true;
    }

    if (sensorStackReady && !aht20MeasurementPending &&
        TimeReached(now, nextSensorTick))
    {
      nextSensorTick = now + kSensorPeriodMs;
      if (AHT20_StartMeasurement())
      {
        aht20MeasurementPending = true;
        aht20MeasurementStartedTick = now;
      }
      else
      {
        sensorStackReady = false;
        nextSensorReconnectTick =
            HAL_GetTick() + kSensorReconnectPeriodMs;
        sensor.valid = false;
        if (runtime.running && HeatingConfigured(settings))
        {
          LatchFault(runtime, FaultCode::Sensor, now);
        }
        displayDirty = true;
      }
    }

    if (aht20MeasurementPending &&
        now - aht20MeasurementStartedTick >= kAht20ConversionMs)
    {
      aht20MeasurementPending = false;
      AHT20_Measurement ahtMeasurement{};
      int32_t bmpTemperatureCentiC = 0;
      if (AHT20_ReadMeasurement(&ahtMeasurement) &&
          BMP280_ReadTemperature(&bmpTemperatureCentiC) &&
          SensorValuesPlausible(ahtMeasurement, bmpTemperatureCentiC))
      {
        sensor.ahtTemperatureCentiC = ahtMeasurement.temperature_centi_c;
        sensor.bmpTemperatureCentiC = bmpTemperatureCentiC;
        sensor.humidityMilliPercent = ahtMeasurement.humidity_milli_percent;
        sensor.valid = true;
      }
      else
      {
        sensorStackReady = false;
        nextSensorReconnectTick =
            HAL_GetTick() + kSensorReconnectPeriodMs;
        sensor.valid = false;
        if (runtime.running && HeatingConfigured(settings))
        {
          LatchFault(runtime, FaultCode::Sensor, now);
        }
      }
      displayDirty = true;
    }

    const int8_t encoderStep = ReadEncoderStep();
    if (encoderStep != 0)
    {
      if (page == UiPage::SetTemperature)
      {
        AdjustOptionalTarget(editValue, encoderStep,
                             kMinimumTargetTemperatureC,
                             kMaximumTargetTemperatureC);
        displayDirty = true;
      }
      else if (page == UiPage::SetHumidity)
      {
        AdjustOptionalTarget(editValue, encoderStep,
                             kMinimumTargetHumidityPercent,
                             kMaximumTargetHumidityPercent);
        displayDirty = true;
      }
      else if (page == UiPage::StartConfirm)
      {
        startSelected = !startSelected;
        displayDirty = true;
      }
    }

    const ButtonEvent buttonEvent = UpdateButton(button, now);
    if (buttonEvent == ButtonEvent::ShortPress)
    {
      if (page == UiPage::Dashboard)
      {
        if (!runtime.running)
        {
          draftSettings = settings;
          editValue = draftSettings.temperatureEnabled
                          ? draftSettings.targetTemperatureC
                          : kTargetDisabled;
          page = UiPage::SetTemperature;
        }
      }
      else if (page == UiPage::SetTemperature)
      {
        draftSettings.temperatureEnabled = editValue != kTargetDisabled;
        if (draftSettings.temperatureEnabled)
        {
          draftSettings.targetTemperatureC =
              static_cast<uint8_t>(editValue);
        }
        editValue = draftSettings.humidityEnabled
                        ? draftSettings.targetHumidityPercent
                        : kTargetDisabled;
        page = UiPage::SetHumidity;
      }
      else if (page == UiPage::SetHumidity)
      {
        draftSettings.humidityEnabled = editValue != kTargetDisabled;
        if (draftSettings.humidityEnabled)
        {
          draftSettings.targetHumidityPercent =
              static_cast<uint8_t>(editValue);
        }
        startSelected = WorkAppearsNeeded(draftSettings, sensor);
        page = UiPage::StartConfirm;
      }
      else
      {
        settings = draftSettings;
        SetDryingOutputs(runtime, false);
        if (settingsStorageReady)
        {
          const StoredSettings settingsToStore{
              settings.targetTemperatureC,
              settings.targetHumidityPercent,
              settings.temperatureEnabled,
              settings.humidityEnabled,
          };
          SettingsStorage_Save(&settingsToStore);
        }
        if (startSelected)
        {
          StartRun(runtime, settings, sensor, fan, now);
        }
        page = UiPage::Dashboard;
      }
      displayDirty = true;
    }
    else if (buttonEvent == ButtonEvent::LongPress)
    {
      if (page == UiPage::Dashboard)
      {
        if (runtime.fault != FaultCode::None)
        {
          if (CanClearFault(runtime, settings, sensor, fan))
          {
            runtime.fault = FaultCode::None;
          }
        }
        else if (runtime.running)
        {
          StopRun(runtime, now);
        }
      }
      else if (page == UiPage::SetTemperature)
      {
        page = UiPage::Dashboard;
      }
      else if (page == UiPage::SetHumidity)
      {
        editValue = draftSettings.temperatureEnabled
                        ? draftSettings.targetTemperatureC
                        : kTargetDisabled;
        page = UiPage::SetTemperature;
      }
      else
      {
        editValue = draftSettings.humidityEnabled
                        ? draftSettings.targetHumidityPercent
                        : kTargetDisabled;
        page = UiPage::SetHumidity;
      }
      displayDirty = true;
    }

    if (UpdateControl(runtime, settings, sensor, fan, now))
    {
      displayDirty = true;
    }

    if (displayReady &&
        (displayDirty || TimeReached(now, nextDisplayTick)))
    {
      RenderScreen(page, editValue, startSelected, settings, draftSettings,
                   sensor, fan, runtime, now);
      displayReady = OLED_IsReady();
      displayDirty = false;
      const uint32_t displayFinishedTick = HAL_GetTick();
      nextDisplayTick = displayFinishedTick + kDisplayPeriodMs;
      if (!displayReady)
      {
        nextOledProbeTick = displayFinishedTick + 2000U;
      }
    }

    HAL_Delay(kLoopDelayMs);
  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef oscillator{};
  RCC_ClkInitTypeDef clocks{};

  oscillator.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  oscillator.HSEState = RCC_HSE_ON;
  oscillator.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  oscillator.HSIState = RCC_HSI_ON;
  oscillator.PLL.PLLState = RCC_PLL_ON;
  oscillator.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  oscillator.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&oscillator) != HAL_OK)
  {
    Error_Handler();
  }

  clocks.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                     RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  clocks.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clocks.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clocks.APB1CLKDivider = RCC_HCLK_DIV2;
  clocks.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&clocks, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  __disable_irq();

  /* Force power-control pins low even if a timer or peripheral has failed. */
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
  TIM2->CCER = 0U;
  TIM2->CR1 = 0U;
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN;
  GPIOA->CRL = (GPIOA->CRL & ~((0x0FU << 0U) | (0x0FU << 8U))) |
               (0x02U << 0U) | (0x02U << 8U);
  GPIOA->BRR = MOTOR_IN1_Pin | MOTOR_IN2_Pin;
  GPIOB->CRH = (GPIOB->CRH & ~0x0FU) | 0x02U;
  if (kXyMosActiveHigh)
  {
    GPIOB->BRR = XY_MOS_IN_Pin;
  }
  else
  {
    GPIOB->BSRR = XY_MOS_IN_Pin;
  }
  while (1)
  {
  }
}
