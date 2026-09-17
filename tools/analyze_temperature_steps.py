import csv,json,sys
from pathlib import Path
p=Path(sys.argv[1]); result=json.loads((p/'result.json').read_text()); assert result['completed']
rows=list(csv.DictReader((p/'samples.csv').open())); out=[]
for e in result['events']:
 if e['event']!='stage_start_command':continue
 phase=e['stage'];target=e['target_c'];r=[x for x in rows if x['phase']==phase];v=[float(x['temperature_c']) for x in r]
 # Setpoint application from first recorded tick showing the new target/running.
 applied=[x for x in rows if float(x['elapsed_s'])>=e['elapsed_s'] and int(x['target_temperature_c'])==target and x['running']=='True']
 t0=int(applied[0]['stm32_tick_ms']); times=[(int(x['stm32_tick_ms'])-t0)/1000 for x in r]
 idx=next((i for i in range(len(v)) if all(abs(t-target)<=.2 for t in v[i:])),None)
 tail=[x for x in r if int(r[-1]['stm32_tick_ms'])-int(x['stm32_tick_ms'])<120000]
 avg=lambda key:sum(float(x[key]) for x in tail)/len(tail)
 out.append(dict(target=target,start=v[0],peak=max(v),overshoot=max(v)-target,first_target_s=next((times[i] for i,t in enumerate(v) if t>=target),None),settling_02_s=times[idx] if idx is not None else None,observation_after_settling_s=times[-1]-times[idx] if idx is not None else None,tail_mean=avg('temperature_c'),tail_min=min(float(x['temperature_c']) for x in tail),tail_max=max(float(x['temperature_c']) for x in tail),tail_duty=avg('output_permille'),max_gap_s=max((int(b['stm32_tick_ms'])-int(a['stm32_tick_ms']))/1000 for a,b in zip(r,r[1:]))))
(p/'metrics.json').write_text(json.dumps(out,indent=2)+'\n'); print(json.dumps(out,indent=2))
