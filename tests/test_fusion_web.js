// Exercise the actual page renderer without network or device access.
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const assert = require('node:assert/strict');
const html = fs.readFileSync(path.join(__dirname, '../esp32_gateway/main/index.html'), 'utf8');
const script = html.split('<script>')[1].split('</script>')[0];
new vm.Script(script); // Check the complete page script syntax.
const renderer = script.slice(script.indexOf('  function showSample('), script.indexOf('  function updateDataBadge('));
const values = {};
const tick = {};
const context = {
  setValue: (id, value, unit) => { values[id] = {value, unit}; },
  number: value => Number.isFinite(value) ? value.toFixed(2) : '--',
  $: id => { assert.equal(id, 'tickValue'); return tick; },
  updateDataBadge: () => {}, showActual: () => {},
};
vm.createContext(context);
vm.runInContext(renderer, context);
context.showSample({valid:true, temperature_c:39.82, humidity_pct:50, ambient_temperature_c:18, stm32_tick_ms:1000});
assert.deepEqual(values.temperatureValue, {value:'39.82',unit:'°C'});
assert.deepEqual(Object.keys(values).sort(), ['humidityValue','temperatureValue']);
context.showSample({valid:false, temperature_c:null, humidity_pct:null, stm32_tick_ms:2000});
assert.equal(values.temperatureValue.value, '--');
assert.equal(values.humidityValue.value, '--');
assert(!/bmpValue|ahtValue|bmp_c|aht_c/.test(html));
assert(html.includes("line('temperature_c'"));
assert(html.includes('id="ambientTemperature"'));
assert(html.includes('id="ambientValue"'));
assert(script.includes('ambient_temperature_c'));
assert(script.includes("integerField('ambientTemperature',-10,50"));
console.log('single-temperature web renderer tests passed');

const controlLogic = script.slice(
  script.indexOf('  function controlMatches('),
  script.indexOf('  function showControlStatus('),
);
const controlContext = {pendingRequest: null, draftRevision: 1, controlDirty: true};
vm.createContext(controlContext);
vm.runInContext(controlLogic, controlContext);
const actual = {
  extended: true,
  running: false,
  temperature_enabled: true,
  target_temperature_c: 40,
  humidity_enabled: true,
  target_humidity_pct: 20,
};
const oldApplied = {
  sequence: 7,
  requested: {...actual},
};
assert.equal(controlContext.controlDraftMatches(oldApplied, actual), false,
             'an old applied request must not overwrite a dirty draft');
controlContext.pendingRequest = {sequence: 8, revision: 1};
const submitted = {
  sequence: 8,
  requested: {...actual, target_temperature_c: 50},
};
const appliedAtNewTarget = {...actual, target_temperature_c: 50};
assert.equal(controlContext.controlDraftMatches(submitted, appliedAtNewTarget), true,
             'the current submitted request may reconcile the draft');
controlContext.draftRevision = 2;
assert.equal(controlContext.controlDraftMatches(submitted, appliedAtNewTarget), false,
             'a newer edit must remain untouched while an older request is pending');
console.log('control draft synchronization tests passed');

const embedded = fs.readFileSync(path.join(__dirname, '../esp32_gateway/main/index_html.h'), 'utf8');
const bytes = Buffer.from([...embedded.matchAll(/0x([0-9a-f]{2})/g)].map(m => parseInt(m[1],16)));
assert.equal(bytes.toString('utf8'), html, 'firmware embedded page must match index.html');
console.log('embedded firmware page matches HTML');
