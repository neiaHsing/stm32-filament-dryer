"""Exercise actual flash storage and local menu with host hardware substitutes."""
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
source = (ROOT / "Core/Src/main.cpp").read_text()


def function(name):
    match = re.search(r'^(?:void|bool) ' + name + r'\([^;]*?\)\n\{', source, re.M)
    assert match, name
    end, depth = match.end(), 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[match.start():end] + '\n'


storage = r'''
#include <assert.h>
#include <stdio.h>
#include "settings_storage.c"
static uint8_t flash[8192];
static unsigned programs, erases;
static bool fail_program;
bool W25Q_Init(SPI_HandleTypeDef *spi) { (void)spi; return true; }
uint32_t W25Q_GetCapacityBytes(void) { return sizeof(flash); }
bool W25Q_Read(uint32_t a,uint8_t *p,uint16_t n) {
  assert(a+n<=sizeof(flash)); memcpy(p,flash+a,n); return true;
}
bool W25Q_PageProgram(uint32_t a,const uint8_t *p,uint16_t n) {
  assert(a+n<=sizeof(flash)); ++programs;
  for (unsigned i=0;i<(fail_program ? 7U : n);++i) flash[a+i]&=p[i];
  return !fail_program;
}
bool W25Q_EraseSector(uint32_t a) {
  assert(a%4096==0 && a+4096<=sizeof(flash));
  memset(flash+a,255,4096); ++erases; return true;
}
static void old_record(unsigned version) {
  memset(flash,255,sizeof(flash));
  memcpy(flash,"DRY1",4); Storage_WriteU32(flash+4,7);
  flash[8]=55; flash[9]=20; flash[10]=version;
  flash[11]=version==1 ? (55^20^0xA5) : 2;
  Storage_WriteU32(flash+12,Storage_Crc32(flash,12));
}
int main(void) {
  StoredSettings s, loaded;
  for (unsigned version=1;version<=2;++version) {
    old_record(version);
    assert(SettingsStorage_Init(NULL) && SettingsStorage_Load(&s));
    assert(s.ambient_temperature_c==22 && s.target_temperature_c==55);
    assert(s.humidity_enabled && s.temperature_enabled==(version==1));
    s.ambient_temperature_c=-10;
    assert(SettingsStorage_Save(&s));
    assert(SettingsStorage_Init(NULL) && SettingsStorage_Load(&loaded));
    assert(loaded.ambient_temperature_c==-10);
    assert(loaded.temperature_enabled==s.temperature_enabled);
    // Damaged latest record must fall back to the old CRC-protected record.
    flash[16+12]^=1;
    assert(SettingsStorage_Init(NULL) && SettingsStorage_Load(&loaded));
    assert(loaded.ambient_temperature_c==22);
  }
  memset(flash,255,sizeof(flash));
  assert(SettingsStorage_Init(NULL) && !SettingsStorage_Load(&loaded));
  s=(StoredSettings){60,30,true,false,22};
  // All room values, including -1 C (not the target OFF sentinel), all flags.
  for (int pass=0;pass<3;++pass) {
    for (int room=-10;room<=50;++room) {
      for (unsigned flags=0;flags<4;++flags) {
        s.ambient_temperature_c=room;
        s.temperature_enabled=flags&1; s.humidity_enabled=flags&2;
        assert(SettingsStorage_Save(&s));
        assert(SettingsStorage_Init(NULL) && SettingsStorage_Load(&loaded));
        assert(loaded.ambient_temperature_c==room);
        assert(loaded.temperature_enabled==s.temperature_enabled);
        assert(loaded.humidity_enabled==s.humidity_enabled);
        assert(loaded.target_temperature_c==60 && loaded.target_humidity_percent==30);
      }
    }
  }
  assert(erases>=2); // Cross both sectors and reuse an old sector.
  unsigned count=programs;
  assert(SettingsStorage_Save(&s) && programs==count);
  s.ambient_temperature_c=-11; assert(!SettingsStorage_Save(&s));
  s.ambient_temperature_c=51; assert(!SettingsStorage_Save(&s));
  s.ambient_temperature_c=21; fail_program=true;
  assert(!SettingsStorage_Save(&s)); fail_program=false;
  assert(SettingsStorage_Init(NULL) && SettingsStorage_Load(&loaded));
  assert(loaded.ambient_temperature_c==50); // Partial write keeps prior record.
  assert(SettingsStorage_Save(&s));
  assert(SettingsStorage_Init(NULL) && SettingsStorage_Load(&loaded));
  assert(loaded.ambient_temperature_c==21);
  puts("ambient flash migration, CRC, partial-write and rollover tests passed");
}
'''

prefix = source[source.index('constexpr uint32_t kLoopDelayMs'):source.index('struct RemoteState')]
menu_start = source.index('    const int8_t encoderStep = ReadEncoderStep();')
menu_end = source.index('    if (UpdateControl(runtime, settings, sensor, fan, now))', menu_start)
menu = r'''
#include "separation_pid.h"
#include "settings_storage.h"
#include <cassert>
#include <cstdio>
#include <cstring>
''' + prefix + r'''
Settings settings, draftSettings;
SensorData sensor;
FanState fan;
RuntimeState runtime;
UiPage page=UiPage::Dashboard;
int16_t editValue=0;
bool startSelected=false, displayDirty=false, settingsStorageReady=true;
uint32_t now=10000;
int button=0, rotation=0, writes=0;
ButtonEvent event=ButtonEvent::None;
StoredSettings saved;
int8_t ReadEncoderStep() { return rotation; }
ButtonEvent UpdateButton(int &,uint32_t) { return event; }
bool SettingsStorage_Save(const StoredSettings *s) { saved=*s; ++writes; return true; }
bool SetDryingOutputs(RuntimeState &,bool) { return true; }
bool StartRun(RuntimeState &r,const Settings &,const SensorData &,const FanState &,uint32_t) {
  r.running=true; return true;
}
bool StopRun(RuntimeState &r,uint32_t) { r.running=false; return true; }
bool CanClearFault(const RuntimeState &,const Settings &,const SensorData &,const FanState &) {
  return true;
}
''' + function('HeatingConfigured') + function('WorkAppearsNeeded') + function('AdjustOptionalTarget') + r'''
void input(ButtonEvent e=ButtonEvent::None,int step=0) {
  event=e; rotation=step;
''' + source[menu_start:menu_end] + r'''
}
int main() {
  const auto press=ButtonEvent::ShortPress, back=ButtonEvent::LongPress;
  sensor.valid=true; sensor.humidityMilliPercent=80000;
  input(press); assert(page==UiPage::SetTemperature);
  input(press); assert(page==UiPage::SetHumidity);
  input(press); assert(page==UiPage::SetAmbient && editValue==22);
  for (int i=0;i<100;++i) input(ButtonEvent::None,-1);
  assert(editValue==-10 && settings.ambientTemperatureC==22);
  for (int i=0;i<100;++i) input(ButtonEvent::None,1);
  assert(editValue==50);
  input(press); assert(page==UiPage::StartConfirm);
  input(back); assert(page==UiPage::SetAmbient && editValue==50);
  input(back); assert(page==UiPage::SetHumidity);
  input(back); assert(page==UiPage::SetTemperature);
  input(back); assert(page==UiPage::Dashboard && settings.ambientTemperatureC==22);
  assert(writes==0); // Cancelled changes never apply or persist.
  input(press); input(press); input(press);
  input(ButtonEvent::None,-1); input(press);
  assert(page==UiPage::StartConfirm && startSelected);
  input(ButtonEvent::None,1); // SAVE ONLY
  input(press);
  assert(page==UiPage::Dashboard && !runtime.running && writes==1);
  assert(settings.ambientTemperatureC==21 && saved.ambient_temperature_c==21);
  input(press); input(press); input(press); input(press); input(press);
  assert(runtime.running && saved.ambient_temperature_c==21);
  input(press); assert(page==UiPage::Dashboard); // No live parameter editing.
  input(back); assert(!runtime.running);
  puts("manual room menu navigation, limits, cancel, save and start tests passed");
}
'''

with tempfile.TemporaryDirectory(prefix='ambient-test-') as tmp:
    path = Path(tmp)
    (path/'stm32f1xx_hal.h').write_text(
        'typedef struct SPI_HandleTypeDef SPI_HandleTypeDef;\n')
    for name, code, compiler, standard, suffix in [
        ('storage', storage, 'cc', 'c11', 'c'),
        ('menu', menu, 'c++', 'c++17', 'cpp'),
    ]:
        file = path/f'{name}.{suffix}'
        file.write_text(code)
        binary = path/name
        subprocess.run([compiler, '-std='+standard, '-Wall', '-Wextra', '-Werror',
                        '-Wno-unused-const-variable', '-I'+tmp,
                        '-I'+str(ROOT/'Core/Inc'), '-I'+str(ROOT/'Core/Src'),
                        str(file), '-o', str(binary)], check=True)
        subprocess.run([str(binary)], check=True)
