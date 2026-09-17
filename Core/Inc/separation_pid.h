#ifndef SEPARATION_PID_H
#define SEPARATION_PID_H
#include <math.h>
struct SeparationPid {
  // Output units are permille. Integral is the slow holding correction.
  float integral = 0, previous = 0, derivative = 0;
  float previousTarget = 0, previousOutput = 0, heatMemory = 0;
  float recoveryHold = 0, predictedTemperature = 0;
  bool ready = false;
  static float clamp(float value, float low, float high) {
    return value < low ? low : (value > high ? high : value);
  }
  // Smoothed full-duty rates from the 2026-09-16 measurement, C/s.
  // The startup transient is excluded; endpoints are held outside calibration.
  static float fullPowerRate(float temperature) {
    const float t[] = {35.5f, 40.5f, 45.5f, 50.5f, 55.5f, 59.5f};
    const float r[] = {0.4533f, 0.4346f, 0.3619f, 0.2761f, 0.2058f, 0.1420f};
    if (temperature <= t[0]) return r[0];
    for (unsigned i = 1; i < 6; ++i) {
      if (temperature <= t[i]) {
        const float fraction = (temperature - t[i-1]) / (t[i] - t[i-1]);
        return r[i-1] + fraction * (r[i] - r[i-1]);
      }
    }
    return r[5];
  }
  void reset() {
    integral = derivative = previousOutput = heatMemory = recoveryHold = 0;
    ready = false;
  }
  float step(float target, float measured, float dt, float kp, float ki,
             float kd, float separation) {
    // dt==0 is a setpoint-only update on the same sensor sample.
    if (dt < 0 || dt > 5.0f || (dt == 0 && (!ready || measured != previous))) {
      reset(); return 0;
    }
    const float error = target - measured;
    const float slope = ready && dt > 0 ? (measured - previous) / dt : 0;
    const bool targetChanged = ready && target != previousTarget;
    if (targetChanged && target < previousTarget) {
      // Discard positive excess correction on a downward setpoint change.
      integral = integral < 0 ? integral : 0;
    }
    // Preserve measured slope and holding correction when the setpoint rises.
    derivative += (dt / (1.0f + dt)) * (slope - derivative);
    heatMemory += (dt / (4.0f + dt)) * (previousOutput - heatMemory);
    recoveryHold = clamp(recoveryHold - dt, 0, 12);
    if (ready && dt > 0 && error > 0.4f && (slope < -0.10f || derivative < -0.05f))
      recoveryHold = 12;
    if (targetChanged) recoveryHold = recoveryHold < 5 ? 5 : recoveryHold;

    const float rate = fullPowerRate(measured);
    const float gain = clamp(0.4346f / rate, 0.8f, 2.0f);
    const float rising = derivative > 0 ? derivative : 0;
    // Full-power cutoff suggests ~1.65 s; include extra margin at high power.
    // Do not subtract a second stored-heat term: measured rise already includes
    // that effect and trial01 showed excessive braking followed by a 0.4 C dip.
    const float horizon = 1.7f + 0.5f * clamp(heatMemory, 0, 1000) * 0.001f;
    predictedTemperature = measured + horizon * rising;
    const float predictedError = target - predictedTemperature;
    const float p = kp * gain * predictedError;
    // Rising-temperature D braking; falling correction is separately capped.
    const float d = -kd * gain * rising;
    // Near the setpoint, catch a falling temperature before error grows.
    // Approximate rate-to-duty conversion from full-duty heating data; cap the
    // transient at 10 percentage points. Large drops still use P + held I.
    const float coolingBoost = error > -0.20f && error <= 2.0f && derivative < 0
        ? clamp(-derivative * 1000.0f / rate, 0, 100)
        : 0;
    // Regulate continuously across the setpoint. Absolute overtemperature
    // shutdown remains independently enforced by UpdateControl.
    constexpr float ceiling = 1000.0f;

    // Only update the slow holding correction once the transient is slow. In
    // particular, do not bank extra heat during a short temperature drop or
    // rising approach.
    if (fabsf(error) <= separation && recoveryHold == 0 &&
        fabsf(derivative) <= 0.10f && !(error > 0 && predictedError <= 0)) {
      const float candidate = clamp(integral + ki * gain * error * dt, -250, 250);
      const float raw = p + candidate + d + coolingBoost;
      if ((raw >= 0 && raw <= ceiling) || (raw > ceiling && error < 0) ||
          (raw < 0 && error > 0)) integral = candidate;
    }
    previous = measured;
    previousTarget = target;
    ready = true;
    previousOutput = clamp(p + integral + d + coolingBoost, 0, ceiling);
    return previousOutput;
  }
};
#endif
