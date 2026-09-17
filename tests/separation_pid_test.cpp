#include "separation_pid.h"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <initializer_list>
static float step(SeparationPid &p, float target, float measured) {
  const float duty = p.step(target, measured, 1, 200, 2.0f, 180, 2);
  assert(std::isfinite(duty) && duty >= 0 && duty <= 1000);
  return duty;
}
int main() {
  // Holding power is an additive target/ambient feedforward term.
  SeparationPid withFeedforward;
  const float target60Feedforward =
      SeparationPid::holdingPower(60, AMBIENT_TEMPERATURE_DEFAULT_C);
  assert(target60Feedforward > 565 && target60Feedforward < 566);
  assert(withFeedforward.step(60, 60, 1, 200, 2, 180, 2) ==
         target60Feedforward);
  assert(std::fabs(SeparationPid::holdingPower(40,24)-289.6107f)<0.1f);
  assert(std::fabs(SeparationPid::holdingPower(60,24)-535.9253f)<0.1f);
  assert(SeparationPid::holdingPower(40,40)==0);
  assert(SeparationPid::holdingPower(70,-10)<=850);
  SeparationPid p;
  assert(step(p,40,29.65f)==1000);
  assert(p.integral==0);
  p.reset();
  const float holding = step(p,40,40);
  p.integral = 30;
  const float higher = step(p,42,40);
  assert(higher>holding+250 && p.integral==30);
  p.derivative=.15f;
  step(p,43,40);
  assert(p.derivative>.07f); // Target changes preserve the measured trend.
  step(p,35,40);
  assert(p.previousOutput==0 && p.integral<=0);

  p.reset();
  step(p,40,40);
  p.integral=25;
  step(p,40,37);
  assert(p.recoveryHold==12 && p.integral==25);
  for (int i=0;i<8;++i) step(p,40,37+i*.3f);
  assert(p.integral==25); // Recovery must not bank additional integral.
  step(p,40,40.31f); // Rising-temperature braking remains bounded.

  // A rising approach inside the separation band still accumulates a
  // reduced integral correction; it is no longer blocked by |derivative|.
  p.reset();
  step(p,40,38.5f);
  const float approachIntegral = p.integral;
  step(p,40,38.8f);
  assert(p.integral > approachIntegral);
  assert(p.derivative > 0.10f);
  assert(p.integral > 0);

  // Output decreases with overshoot, including the holding prior.
  for (float target : {40.0f, 65.0f, 70.0f}) {
    SeparationPid below, above;
    const float left = step(below,target,target+.29f);
    const float right = step(above,target,target+.31f);
    assert(right>=0 && right<=left);
    assert(step(above,target,target+5)==0); // Large overshoot still removes heat.
  }
  p.reset();
  step(p,40,40);
  step(p,40,39);
  const float proportionalOnly=200 + SeparationPid::holdingPower(40);
  assert(p.previousOutput<=proportionalOnly+100.1f); // Falling boost is bounded.
  p.reset();
  step(p,40,40);
  const float catchFall=step(p,40,39.95f);
  assert(catchFall>20);
  assert(catchFall < SeparationPid::holdingPower(40) + 111);

  p.reset();
  for(int i=0;i<300;++i) assert(step(p,60,30)==1000);
  assert(p.integral==0);
  for(int i=0;i<5000;++i) step(p,40,39.8f);
  assert(p.integral<=250 && p.integral>=-250);
  assert(p.step(40,39,0,180,1.5f,160,2)==0 && !p.ready);
  assert(p.step(40,39,6,180,1.5f,160,2)==0 && !p.ready);
  step(p,40,40);
  p.derivative=.1f;
  const float savedSlope=p.derivative;
  const float savedMemory=p.heatMemory;
  const float savedIntegral=p.integral;
  assert(p.step(42,40,0,200,2,180,2)>0);
  assert(p.ready && p.derivative==savedSlope && p.heatMemory==savedMemory && p.integral==savedIntegral);

  // Independent lagged thermal plants: regression checks, not empirical claims.
  for (float lag : {1.0f, 3.0f, 5.0f}) {
    p.reset();
    float measured=29.65f, delivered=0, peak=measured;
    for(int i=0;i<600;++i) {
      float u=step(p,40,measured)/1000;
      delivered+=(u-delivered)/(lag+1);
      measured+=.65f*delivered-.0142f*(measured-29.65f);
      if(measured>peak)peak=measured;
    }
    std::printf("plant lag=%.1f peak=%.3f final=%.3f\n",lag,peak,measured);
    assert(peak<41 && std::fabs(measured-40)<.25f);
    peak=measured;
    for(int i=0;i<240;++i) {
      float u=step(p,42,measured)/1000;
      delivered+=(u-delivered)/(lag+1);
      measured+=.65f*delivered-.0142f*(measured-29.65f);
      if(measured>peak)peak=measured;
    }
    assert(peak<42.6f && std::fabs(measured-42)<.25f);
    measured-=3; // A bounded thermal disturbance, at an unchanged setpoint.
    peak=measured;
    for(int i=0;i<240;++i) {
      float u=step(p,42,measured)/1000;
      delivered+=(u-delivered)/(lag+1);
      measured+=.65f*delivered-.0142f*(measured-29.65f);
      if(measured>peak)peak=measured;
    }
    std::printf("  42 C recovery peak=%.3f final=%.3f\n",peak,measured);
    assert(peak<42.6f && std::fabs(measured-42)<.25f);
  }
  assert(SeparationPid::fullPowerRate(35)>SeparationPid::fullPowerRate(55));
  std::puts("controller tests passed");
}
