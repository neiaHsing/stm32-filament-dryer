#include "main.h"

#include "aht20.h"
#include "bmp280.h"
#include "driver_oled.h"
#include "gpio.h"
#include "i2c.h"
#include "remote_control.h"
#include "settings_storage.h"
#include "separation_pid.h"
#include "spi.h"
#include "telemetry.h"
#include "temperature_fusion.h"

#include <cstring>

extern "C" {
volatile float g_pidKp = 160.0f;
volatile float g_pidKi = 2.0f;
volatile float g_pidKd = 180.0f;
volatile float g_pidSeparation = 2.0f;
}

namespace
{
constexpr uint32_t kLoopDelayMs = 2U;
constexpr uint32_t kButtonDebounceMs = 25U;
constexpr uint32_t kButtonLongPressMs = 1000U;
constexpr uint32_t kSensorPeriodMs = 1000U;
constexpr uint32_t kSensorReconnectPeriodMs = 2000U;
constexpr uint32_t kAht20ConversionMs = 85U;
constexpr uint32_t kDisplayPeriodMs = 1000U;
constexpr uint32_t kRemoteLeaseMs = 5000U;
constexpr uint32_t kRemoteRunStartupGuardMs = 5000U;
constexpr uint8_t kMaximumRemoteCommandsPerLoop = 4U;
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
constexpr uint32_t kHeaterLedFastPeriodMs = 160U;
constexpr uint32_t kHeaterLedSlowPeriodMs = 4000U;
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
constexpr uint8_t kEmptyRectangleGlyph[16] = {
    0xE0U, 0x20U, 0x20U, 0x20U, 0x20U, 0x20U, 0xE0U, 0x00U,
    0x07U, 0x04U, 0x04U, 0x04U, 0x04U, 0x04U, 0x07U, 0x00U,
};
constexpr uint8_t kSolidRectangleGlyph[16] = {
    0xE0U, 0xE0U, 0xE0U, 0xE0U, 0xE0U, 0xE0U, 0xE0U, 0x00U,
    0x07U, 0x07U, 0x07U, 0x07U, 0x07U, 0x07U, 0x07U, 0x00U,
};

enum class UiPage : uint8_t
{
  Dashboard,
  SetTemperature,
  SetHumidity,
  SetAmbient,
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

enum class RemoteResult : uint8_t
{
  None = 0,
  Applied = 1,
  BadRange = 2,
  Stale = 3,
  FaultActive = 4,
  Fan = 5,
  Sensor = 6,
  Overtemperature = 7,
  ClearRejected = 8,
  LeaseExpired = 9,
  StorageFailed = 10,
  PersistRequiresStop = 11,
};

struct Settings
{
  uint8_t targetTemperatureC = kDefaultTargetTemperatureC;
  uint8_t targetHumidityPercent = kDefaultTargetHumidityPercent;
  bool temperatureEnabled = true;
  bool humidityEnabled = true;
  int8_t ambientTemperatureC = AMBIENT_TEMPERATURE_DEFAULT_C;
};

struct SensorData
{
  uint32_t sampleTickMs = 0U;
  int32_t temperatureCentiC = 0;
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
  uint16_t heaterPermille = 0;
  SeparationPid pid;
  uint32_t pidTick = 0;
  int32_t pidTarget = 0;
  bool motorOn = false;
  bool dryingNeeded = true;
  bool remoteOwned = false;
  FaultCode fault = FaultCode::None;
  uint32_t runStartTick = 0U;
  uint32_t elapsedSeconds = 0U;
};

struct RemoteState
{
  bool hasCommand = false;
  RemoteControlCommand lastCommand{};
  RemoteResult lastCommandResult = RemoteResult::None;
  uint32_t ackSession = 0U;
  uint32_t ackSequence = 0U;
  RemoteResult ackResult = RemoteResult::None;
  uint32_t lastHeartbeatTick = 0U;
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

bool SetHeaterDuty(RuntimeState &runtime, uint16_t duty)
{
  if (duty > 1000U) duty = 1000U;
  if (runtime.heaterPermille == duty) return false;
  runtime.heaterPermille = duty;
  runtime.heaterOn = duty != 0;
  // Preloaded CCR takes effect at the next 10 Hz period, without restarting it.
  TIM4->CCR3 = ((TIM4->ARR + 1U) * duty) / 1000U;
  if (duty == 0U) TIM4->EGR = TIM_EGR_UG; // Immediate safety shutdown.
  if (duty == 0U)
  {
    /* PC13 is active low; turn the indicator off immediately on shutdown. */
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
  }
  return true;
}

void UpdateHeaterIndicator(const RuntimeState &runtime, uint32_t now)
{
  if (runtime.heaterPermille == 0U)
  {
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    return;
  }

  const uint32_t periodRange =
      kHeaterLedSlowPeriodMs - kHeaterLedFastPeriodMs;
  const uint32_t period =
      kHeaterLedSlowPeriodMs -
      (periodRange * runtime.heaterPermille) / 1000U;
  const bool ledOn = (now % period) < (period / 2U);
  /* PC13 is active low, so the first half of each period is lit. */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin,
                   ledOn ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

bool SetHeaterOutput(RuntimeState &runtime, bool enabled)
{
  if (!enabled) runtime.pid.reset();
  return SetHeaterDuty(runtime, enabled ? 1000U : 0U);
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
  runtime.remoteOwned = false;
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
  return sensor.temperatureCentiC <
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

  /* Always reconcile the physical PWM outputs before beginning a new run. */
  SetDryingOutputs(runtime, false);
  runtime.running = true;
  runtime.remoteOwned = false;
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
                   uint32_t now);

bool SameRemoteCommand(const RemoteControlCommand &left,
                       const RemoteControlCommand &right)
{
  return left.session == right.session &&
         left.sequence == right.sequence &&
         left.seen_tick_ms == right.seen_tick_ms &&
         left.run == right.run &&
         left.temperature_enabled == right.temperature_enabled &&
         left.target_temperature_c == right.target_temperature_c &&
         left.humidity_enabled == right.humidity_enabled &&
         left.target_humidity_percent == right.target_humidity_percent &&
         left.ambient_temperature_c == right.ambient_temperature_c &&
         left.clear_fault == right.clear_fault &&
         left.persist == right.persist &&
         left.pid_update == right.pid_update &&
         left.pid_kp_tenths == right.pid_kp_tenths &&
         left.pid_ki_hundredths == right.pid_ki_hundredths &&
         left.pid_kd_tenths == right.pid_kd_tenths &&
         left.values_in_range == right.values_in_range;
}

RemoteResult ApplyRemotePidCommand(const RemoteControlCommand &command,
                                   RuntimeState &runtime,
                                   RemoteState &remote)
{
  if (!command.values_in_range)
  {
    return RemoteResult::BadRange;
  }
  g_pidKp = command.pid_kp_tenths / 10.0f;
  g_pidKi = command.pid_ki_hundredths / 100.0f;
  g_pidKd = command.pid_kd_tenths / 10.0f;
  runtime.pid.reset();
  return RemoteResult::Applied;
}

RemoteResult MapStartFailure(FaultCode fault)
{
  if (fault == FaultCode::Fan)
  {
    return RemoteResult::Fan;
  }
  if (fault == FaultCode::Sensor)
  {
    return RemoteResult::Sensor;
  }
  if (fault == FaultCode::Overtemperature)
  {
    return RemoteResult::Overtemperature;
  }
  return RemoteResult::FaultActive;
}

RemoteResult ApplyRemoteCommand(const RemoteControlCommand &command,
                                Settings &settings,
                                RuntimeState &runtime,
                                const SensorData &sensor,
                                const FanState &fan,
                                bool settingsStorageReady,
                                RemoteState &remote,
                                uint32_t now,
                                uint32_t remoteRunAllowedTick)
{
  if (command.pid_update)
  {
    RemoteResult result = RemoteResult::None;
    if (remote.hasCommand &&
        command.session == remote.lastCommand.session &&
        command.sequence == remote.lastCommand.sequence)
    {
      if (!SameRemoteCommand(command, remote.lastCommand))
      {
        result = RemoteResult::Stale;
      }
      else
      {
        result = remote.lastCommandResult;
      }
    }
    else
    {
      result = ApplyRemotePidCommand(command, runtime, remote);
    }
    remote.hasCommand = true;
    remote.lastCommand = command;
    remote.lastCommandResult = result;
    remote.ackSession = command.session;
    remote.ackSequence = command.sequence;
    remote.ackResult = result;
    return result;
  }
  if (remote.hasCommand &&
      command.session == remote.lastCommand.session &&
      command.sequence == remote.lastCommand.sequence)
  {
    /* Repeated transport frames acknowledge the original result. They never
       repeat a flash write or control transition. A byte-for-byte equivalent
       accepted RUN frame also proves that the remote controller is alive. */
    if (!SameRemoteCommand(command, remote.lastCommand))
    {
      remote.ackSession = command.session;
      remote.ackSequence = command.sequence;
      remote.ackResult = RemoteResult::Stale;
      return RemoteResult::Stale;
    }
    if (command.run &&
        (remote.lastCommandResult == RemoteResult::Applied ||
         remote.lastCommandResult == RemoteResult::StorageFailed) &&
        runtime.running && runtime.remoteOwned)
    {
      remote.lastHeartbeatTick = now;
    }
    remote.ackSession = command.session;
    remote.ackSequence = command.sequence;
    remote.ackResult = remote.lastCommandResult;
    return remote.lastCommandResult;
  }

  if (remote.hasCommand &&
      command.session == remote.lastCommand.session &&
      static_cast<int32_t>(command.sequence -
                           remote.lastCommand.sequence) < 0)
  {
    /* An older command may be reported, but it cannot replace the cached
       command/result or roll the controller state back. */
    remote.ackSession = command.session;
    remote.ackSequence = command.sequence;
    remote.ackResult = RemoteResult::Stale;
    return RemoteResult::Stale;
  }

  RemoteResult result = RemoteResult::Applied;
  if (!command.values_in_range)
  {
    result = RemoteResult::BadRange;
  }
  else if (command.run && !TimeReached(now, remoteRunAllowedTick))
  {
    /* Consume and acknowledge a RUN received during startup as stale. Its
       sequence therefore cannot start the machine later when the guard
       expires; the controller must issue a fresh RUN command. STOP/settings
       commands remain available throughout the guard interval. */
    result = RemoteResult::Stale;
  }
  else if (command.run && command.persist)
  {
    /* Flash erase/program can block the safety loop for seconds. Require a
       separate stopped-state save before a remote start. */
    result = RemoteResult::PersistRequiresStop;
  }
  else if (command.run && now - command.seen_tick_ms > kRemoteLeaseMs)
  {
    result = RemoteResult::Stale;
  }
  else
  {
    settings.targetTemperatureC = command.target_temperature_c;
    settings.temperatureEnabled = command.temperature_enabled;
    settings.targetHumidityPercent = command.target_humidity_percent;
    settings.humidityEnabled = command.humidity_enabled;
    settings.ambientTemperatureC = command.ambient_temperature_c;

    if (!command.run)
    {
      StopRun(runtime, now);
    }

    if (command.clear_fault && runtime.fault != FaultCode::None)
    {
      if (CanClearFault(runtime, settings, sensor, fan))
      {
        runtime.fault = FaultCode::None;
      }
      else
      {
        result = RemoteResult::ClearRejected;
      }
    }

    if (result == RemoteResult::Applied && command.run)
    {
      if (runtime.fault != FaultCode::None)
      {
        result = RemoteResult::FaultActive;
      }
      else if (!runtime.running &&
               !StartRun(runtime, settings, sensor, fan, now))
      {
        result = MapStartFailure(runtime.fault);
      }

      if (result == RemoteResult::Applied)
      {
        runtime.remoteOwned = true;
        remote.lastHeartbeatTick = now;
      }
    }

    /* Apply a lowered/disabled target before a requested flash write can
       block the main loop. All normal safety gates remain in UpdateControl. */
    UpdateControl(runtime, settings, sensor, fan, now);

    if (command.persist)
    {
      const StoredSettings settingsToStore{
          settings.targetTemperatureC,
          settings.targetHumidityPercent,
          settings.temperatureEnabled,
          settings.humidityEnabled,
          settings.ambientTemperatureC,
      };
      if ((!settingsStorageReady ||
           !SettingsStorage_Save(&settingsToStore)) &&
          result == RemoteResult::Applied)
      {
        result = RemoteResult::StorageFailed;
      }
    }
  }

  remote.hasCommand = true;
  remote.lastCommand = command;
  remote.lastCommandResult = result;
  remote.ackSession = command.session;
  remote.ackSequence = command.sequence;
  remote.ackResult = result;
  return result;
}

uint8_t BuildRemoteStateBits(const Settings &settings,
                             const RuntimeState &runtime,
                             const FanState &fan)
{
  return static_cast<uint8_t>(
      (runtime.running ? 1U << 0U : 0U) |
      (runtime.heaterOn ? 1U << 1U : 0U) |
      (runtime.motorOn ? 1U << 2U : 0U) |
      (settings.temperatureEnabled ? 1U << 3U : 0U) |
      (settings.humidityEnabled ? 1U << 4U : 0U) |
      (fan.running ? 1U << 5U : 0U) |
      (fan.sampleValid ? 1U << 6U : 0U) |
      (runtime.remoteOwned ? 1U << 7U : 0U));
}

bool SendTelemetrySnapshot(uint32_t tick, const SensorData &sensor,
                           const Settings &settings,
                           const RuntimeState &runtime, const FanState &fan,
                           const RemoteState &remote)
{
  return Telemetry_Send(
      tick, sensor.temperatureCentiC,
      sensor.humidityMilliPercent, sensor.valid,
      BuildRemoteStateBits(settings, runtime, fan),
      runtime.heaterPermille, settings.targetTemperatureC,
      settings.targetHumidityPercent, static_cast<uint8_t>(runtime.fault),
      settings.ambientTemperatureC,
      remote.ackSession, remote.ackSequence,
      static_cast<uint8_t>(remote.ackResult),
      RemoteControl_GetRxErrorCount());
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
  const int32_t controlTemperature = sensor.temperatureCentiC;
  const bool targetChanged = runtime.pidTarget != targetTemperature;
  if (targetChanged)
  {
    runtime.pidTarget = targetTemperature;
  }
  const bool newSample = runtime.pidTick != sensor.sampleTickMs;
  if (!runtime.pid.ready || targetChanged || newSample)
  {
    // Follow the actual sensor cadence, never differentiate a repeated sample.
    const uint32_t pidElapsed = sensor.sampleTickMs - runtime.pidTick;
    const float dt = runtime.pid.ready ? (newSample ? pidElapsed / 1000.0f : 0.0f) : 1.0f;
    runtime.pidTick = sensor.sampleTickMs;
    const float duty = runtime.pid.step(targetTemperature / 100.0f,
        controlTemperature / 100.0f, dt, g_pidKp, g_pidKi, g_pidKd,
        g_pidSeparation);
    changed = SetHeaterDuty(runtime, static_cast<uint16_t>(duty + 0.5f)) || changed;
  }
  // Keep circulation running throughout regulation, including zero heat demand.
  changed = SetMotorOutput(runtime, true) || changed;
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

void PutDutyBar(char line[17], uint16_t dutyPermille)
{
  constexpr uint8_t kDutyBarWidth = 16U;
  const uint8_t filled = static_cast<uint8_t>(
      (static_cast<uint32_t>(dutyPermille) * kDutyBarWidth + 500U) / 1000U);

  for (uint8_t index = 0U; index < kDutyBarWidth; ++index)
  {
    line[index] = index < filled ? '#' : '.';
  }
}

void RenderDashboard(char lines[4][17], const Settings &settings,
                     const SensorData &sensor, const FanState &fan,
                     const RuntimeState &runtime, uint32_t now)
{
  if (sensor.valid)
  {
    PutText(lines[0], 0U, "T:");
    PutTemperature(lines[0], 2U, sensor.temperatureCentiC);
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

  PutDutyBar(lines[1], runtime.heaterPermille);

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
  else if (page == UiPage::SetAmbient)
  {
    PutText(lines[0], 0U, "ROOM TEMP CAL");
    PutText(lines[1], 0U, "ROOM:");
    PutTemperature(lines[1], 7U, editValue * 100);
    lines[1][12] = 'C';
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
      if (page == UiPage::Dashboard && row == 1U)
      {
        for (uint8_t index = 0U; index < 16U; ++index)
        {
          if (!g_displayCacheValid ||
              g_displayCache[row][index] != lines[row][index])
          {
            OLED_PutGlyph(index, 2U, lines[row][index] == '#'
                                       ? kSolidRectangleGlyph
                                       : kEmptyRectangleGlyph);
          }
        }
      }
      else
      {
        OLED_PrintString(0U, static_cast<uint8_t>(row * 2U), lines[row]);
      }
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
  const uint32_t remoteRunAllowedTick =
      HAL_GetTick() + kRemoteRunStartupGuardMs;
  MX_GPIO_Init();
  g_encoderPreviousState = ReadEncoderState();
  g_encoderTransitionAccumulator = 0;
  g_encoderPendingSteps = 0;
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  RemoteControl_Init();
  Telemetry_Init();

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
    settings.ambientTemperatureC = storedSettings.ambient_temperature_c;
  }

  bool sensorStackReady = InitializeSensorStack(false);

  SensorData sensor;
  FanState fan;
  RemoteState remote;
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
  uint32_t lastTelemetryTick = initialTick;
  bool displayDirty = true;

  while (1)
  {
    const uint32_t now = HAL_GetTick();

    RemoteControlCommand remoteCommand{};
    for (uint8_t commandCount = 0U;
         commandCount < kMaximumRemoteCommandsPerLoop &&
         RemoteControl_TryRead(&remoteCommand);
         ++commandCount)
    {
      const uint32_t commandNow = HAL_GetTick();
      ApplyRemoteCommand(remoteCommand, settings, runtime, sensor, fan,
                         settingsStorageReady, remote, commandNow,
                         remoteRunAllowedTick);
      page = UiPage::Dashboard;
      draftSettings = settings;
      editValue = settings.temperatureEnabled
                      ? settings.targetTemperatureC
                      : kTargetDisabled;
      startSelected = WorkAppearsNeeded(settings, sensor);
      displayDirty = true;
    }

    const uint32_t leaseNow = HAL_GetTick();
    if (runtime.running && runtime.remoteOwned &&
        leaseNow - remote.lastHeartbeatTick >= kRemoteLeaseMs)
    {
      StopRun(runtime, leaseNow);
      remote.lastCommandResult = RemoteResult::LeaseExpired;
      remote.ackSession = remote.lastCommand.session;
      remote.ackSequence = remote.lastCommand.sequence;
      remote.ackResult = RemoteResult::LeaseExpired;
      displayDirty = true;
    }

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
      // Keep a fixed cadence; skip missed slots instead of burst sampling.
      nextSensorTick += kSensorPeriodMs;
      if (TimeReached(now, nextSensorTick))
      {
        nextSensorTick = now + kSensorPeriodMs;
      }
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
        sensor.temperatureCentiC = FuseTemperature(
            sensor.ahtTemperatureCentiC, sensor.bmpTemperatureCentiC);
        sensor.humidityMilliPercent = ahtMeasurement.humidity_milli_percent;
        sensor.sampleTickMs = aht20MeasurementStartedTick;
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
      SendTelemetrySnapshot(aht20MeasurementStartedTick, sensor, settings,
                            runtime, fan, remote);
      lastTelemetryTick = HAL_GetTick();
      displayDirty = true;
    }

    if (!sensorStackReady &&
        HAL_GetTick() - lastTelemetryTick >= kSensorPeriodMs)
    {
      lastTelemetryTick = HAL_GetTick();
      SensorData invalidSensor{};
      SendTelemetrySnapshot(lastTelemetryTick, invalidSensor, settings,
                            runtime, fan, remote);
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
      else if (page == UiPage::SetAmbient)
      {
        const int16_t next = editValue + encoderStep;
        if (next >= AMBIENT_TEMPERATURE_MIN_C &&
            next <= AMBIENT_TEMPERATURE_MAX_C)
        {
          editValue = next;
        }
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
        editValue = draftSettings.ambientTemperatureC;
        page = UiPage::SetAmbient;
      }
      else if (page == UiPage::SetAmbient)
      {
        draftSettings.ambientTemperatureC = static_cast<int8_t>(editValue);
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
              settings.ambientTemperatureC,
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
      else if (page == UiPage::SetAmbient)
      {
        editValue = draftSettings.humidityEnabled
                        ? draftSettings.targetHumidityPercent
                        : kTargetDisabled;
        page = UiPage::SetHumidity;
      }
      else
      {
        editValue = draftSettings.ambientTemperatureC;
        page = UiPage::SetAmbient;
      }
      displayDirty = true;
    }

    if (UpdateControl(runtime, settings, sensor, fan, now))
    {
      displayDirty = true;
    }
    UpdateHeaterIndicator(runtime, now);

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
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN | RCC_APB1ENR_TIM4EN;
  (void)RCC->APB1ENR;
  TIM4->CCER = 0U;
  TIM4->CR1 = 0U;
  TIM2->CCER = 0U;
  TIM2->CR1 = 0U;
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN;
  (void)RCC->APB2ENR;
  GPIOA->BRR = MOTOR_IN1_Pin | MOTOR_IN2_Pin;
  GPIOB->BRR = XY_MOS_IN_Pin;
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
