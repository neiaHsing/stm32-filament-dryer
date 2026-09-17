"""Run real control functions on the host, substituting only hardware outputs.

Extract from main.cpp so tests exercise current safety/control code rather than
copies. Hardware IO and the main event loop are not executed.
"""
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
source = (ROOT / 'Core/Src/main.cpp').read_text()

def function(name):
    match = re.search(r'^(?:bool|void|int32_t) ' + name + r'\([^;]*?\)\n\{', source, re.M)
    assert match, name
    start = match.start()
    pos = match.end()
    depth = 1
    while depth:
        depth += (source[pos] == '{') - (source[pos] == '}')
        pos += 1
    return source[start:pos] + '\n'

prefix = source[source.index('constexpr uint32_t kLoopDelayMs'):source.index('struct RemoteState')]
code = '''#include "temperature_fusion.h"
#include "separation_pid.h"
#include "ambient_temperature.h"
#include "aht20.h"
#include <cassert>
#include <cmath>
#include <cstdio>
#include <initializer_list>
float g_pidKp=200, g_pidKi=2, g_pidKd=180, g_pidSeparation=2;
''' + prefix + '''
bool SetHeaterDuty(RuntimeState &r, uint16_t duty) {
    r.heaterPermille=duty; r.heaterOn=duty!=0; return true;
}
bool SetMotorOutput(RuntimeState &r, bool enabled) {
    r.motorOn=enabled; return true;
}
'''
for name in ['HeatingConfigured', 'SetHeaterOutput', 'SetDryingOutputs',
             'UpdateElapsedTime', 'StopRun', 'LatchFault', 'MaximumTemperature',
             'WorkAppearsNeeded', 'StartRun', 'SensorValuesPlausible',
             'CanClearFault', 'UpdateControl']:
    code += function(name)
code += r'''
SensorData sample(int32_t aht, int32_t bmp) {
    SensorData s; s.ahtTemperatureCentiC=aht; s.bmpTemperatureCentiC=bmp;
    s.temperatureCentiC=FuseTemperature(aht,bmp); s.valid=true;
    s.sampleTickMs=10000; return s;
}
int main() {
    assert(FuseTemperature(3970,4010)==3982);
    assert(FuseTemperature(-3970,-4010)==-3982);
    assert(FuseTemperature(0,5)==2);
    assert(FuseTemperature(0,-5)==-2);
    assert(FuseTemperature(-1,1)==0);
    assert(FuseTemperature(-4000,-4000)==-4000);
    assert(FuseTemperature(8500,8500)==8500);
    Settings settings; settings.humidityEnabled=false; settings.targetTemperatureC=40;
    FanState fan; fan.sampleValid=true; fan.running=true;
    auto s=sample(3970,4010);
    RuntimeState run; run.running=true;
    assert(WorkAppearsNeeded(settings,s)); // max would incorrectly say target reached.
    UpdateControl(run,settings,s,fan,10000);
    SeparationPid expected;
    const auto duty=expected.step(40,39.82f,1,200,2,180,2);
    assert(run.heaterPermille==static_cast<uint16_t>(duty+0.5f));
    // Ambient participates in the holding-power feedforward.
    for (int ambient : {-10, 22, 50}) {
        settings.ambientTemperatureC=ambient;
        settings.targetTemperatureC=60;
        run={}; run.running=true; s=sample(6000,6000);
        UpdateControl(run,settings,s,fan,10000);
        const auto expected = SeparationPid::holdingPower(60, ambient);
        assert(run.heaterPermille==static_cast<uint16_t>(expected+0.5f));
    }
    for (bool firstHot : {false,true}) {
        s=firstHot?sample(7500,4000):sample(4000,7500);
        assert(s.temperatureCentiC<7500);
        run={}; run.running=true; run.heaterOn=true; run.heaterPermille=1000;
        UpdateControl(run,settings,s,fan,10000);
        assert(run.fault==FaultCode::Overtemperature);
        assert(!run.running && !run.heaterOn && run.heaterPermille==0);
        run={}; assert(!StartRun(run,settings,s,fan,10000));
        assert(run.fault==FaultCode::Overtemperature);
        s=firstHot?sample(7000,4000):sample(4000,7000);
        assert(!CanClearFault(run,settings,s,fan));
        s=sample(6999,6999); assert(CanClearFault(run,settings,s,fan));
    }
    s=sample(3970,4010); s.valid=false;
    run={}; run.running=true; run.heaterOn=true; run.heaterPermille=1000;
    UpdateControl(run,settings,s,fan,10000);
    assert(run.fault==FaultCode::Sensor && !run.running && !run.heaterOn);
    assert(run.heaterPermille==0 && !CanClearFault(run,settings,s,fan));
    AHT20_Measurement aht{}; aht.temperature_centi_c=4000;
    assert(SensorValuesPlausible(aht,4000));
    assert(!SensorValuesPlausible(aht,8501));
    aht.temperature_centi_c=8501; assert(!SensorValuesPlausible(aht,4000));
    puts("temperature fusion and control safety tests passed");
}
'''
# Integration contracts: both sensor reads gate validity; same feedback feeds LCD/SPI.
assert 'AHT20_ReadMeasurement(&ahtMeasurement) &&' in source
assert 'BMP280_ReadTemperature(&bmpTemperatureCentiC) &&' in source
assert 'sensor.temperatureCentiC = FuseTemperature(' in source
assert 'PutTemperature(lines[0], 2U, sensor.temperatureCentiC)' in source
assert 'tick, sensor.temperatureCentiC,' in function('SendTelemetrySnapshot')
assert 'kAht20ConversionMs = 85U' in source
with tempfile.TemporaryDirectory(prefix='fusion-test-') as tmp:
    (Path(tmp)/'stm32f1xx_hal.h').write_text('typedef struct I2C_HandleTypeDef I2C_HandleTypeDef;\n')
    cpp = Path(tmp)/'test.cpp'; cpp.write_text(code)
    binary=Path(tmp)/'test'
    subprocess.run(['c++','-std=c++17','-Wall','-Wextra','-Werror',
                    '-Wno-unused-const-variable','-I'+tmp,'-I'+str(ROOT/'Core/Inc'),
                    str(cpp),'-o',str(binary)],check=True)
    subprocess.run([str(binary)],check=True)
