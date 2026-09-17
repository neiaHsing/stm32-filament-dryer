"""Check gateway serialization using the actual hardware-independent C code."""
from pathlib import Path
import csv
import json
import re
import subprocess
import tempfile
ROOT = Path(__file__).resolve().parents[1]
source=(ROOT/'esp32_gateway/main/web_server.c').read_text()
json_functions=source[source.index('static const char *json_bool'):source.index('static esp_err_t send_json')]
# CSV sensor block is tested with the same valid/invalid frames as WebSocket JSON.
csv_block=source[source.index('        char sensor_values[80];'):source.index('        if (frame->extended) {', source.index('        char sensor_values[80];'))]
program='''#include "telemetry_store.h"
#include <stdio.h>
#include <inttypes.h>
''' + json_functions + '''
int main(void) {
    telemetry_sample_t sample={0};
    sample.frame.extended=true;
    sample.frame.temperature_c=39.82f;
    sample.frame.humidity_percent=50.0f;
    for (int valid=1; valid>=0; --valid) {
        sample.frame.valid=valid;
        char buffer[1024];
        sample_json(buffer,sizeof(buffer),&sample);
        puts(buffer);
        const telemetry_frame_t *frame=&sample.frame;
''' + csv_block + '''
        puts(sensor_values);
    }
}
'''
with tempfile.TemporaryDirectory(prefix='fusion-serialize-') as tmp:
    p=Path(tmp)
    (p/'esp_err.h').write_text('typedef int esp_err_t;\n')
    (p/'test.c').write_text(program)
    subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-I'+tmp,
                    '-I'+str(ROOT/'esp32_gateway/main'),str(p/'test.c'),
                    str(ROOT/'esp32_gateway/main/telemetry_protocol.c'),
                    '-o',str(p/'test')],check=True)
    lines=subprocess.check_output([str(p/'test')],text=True).splitlines()
    good=json.loads(lines[0]); bad=json.loads(lines[2])
    assert good['temperature_c']==39.82 and bad['temperature_c'] is None
    assert good['valid'] is True and bad['valid'] is False
    for row in (good,bad):
        assert 'aht_c' not in row and 'bmp_c' not in row
    assert next(csv.reader([lines[1]]))==['39.820','50.000','1']
    assert next(csv.reader([lines[3]]))==['','','0']
print('gateway single-temperature JSON and CSV tests passed')
