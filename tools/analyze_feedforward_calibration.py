"""Analyze completed 24 C ambient calibration; use device-time weighted tails."""
import csv,json,hashlib
from pathlib import Path
root=Path('outputs/feedforward_calibration_24c')
result=json.loads((root/'result.json').read_text())
assert result['completed'], 'Experiment incomplete; do not fit'
rows=list(csv.DictReader((root/'samples.csv').open()))
metrics=[]
for target,phase in [(40,'stage1_room_to_40_hold'),(60,'stage2_40_to_60_hold')]:
 r=[x for x in rows if x['phase']==phase]
 end=int(r[-1]['stm32_tick_ms'])/1000
 start=end-120
 segments=[]
 for a,b in zip(r,r[1:]):
  ta,tb=int(a['stm32_tick_ms'])/1000,int(b['stm32_tick_ms'])/1000
  dt=max(0,min(tb,end)-max(ta,start))
  if dt: segments.append((a,dt))
 total=sum(dt for _,dt in segments)
 avg=lambda key:sum(float(x[key])*dt for x,dt in segments)/total
 temps=[float(x['temperature_c']) for x,_ in segments]
 first=lambda value:next((float(x['stage_elapsed_s']) for x in r if float(x['temperature_c'])>=value),None)
 m=dict(target=target,samples=len(r),duration_s=float(r[-1]['stage_elapsed_s']),first_target_s=first(target),tail_mean_c=avg('temperature_c'),tail_min_c=min(temps),tail_max_c=max(temps),duty_permille=avg('output_permille'),peak_c=max(float(x['temperature_c']) for x in r),tail_duration_s=total,max_sample_gap_s=max((int(b['stm32_tick_ms'])-int(a['stm32_tick_ms']))/1000 for a,b in zip(r,r[1:])))
 m['overshoot_c']=m['peak_c']-target
 if target==40: m['38_to_40_s']=None if first(40) is None else first(40)-first(38)
 m['reference22_permille']=m['duty_permille']*(target-22)/(target-24)
 metrics.append(m)
slope=(metrics[1]['reference22_permille']-metrics[0]['reference22_permille'])/20
offset=40-metrics[0]['reference22_permille']/slope
out=dict(ambient_c=24,tail_seconds=120,stages=metrics,slope=slope,offset=offset)
(root/'metrics.json').write_text(json.dumps(out,indent=2)+'\n')
print(json.dumps(out,indent=2))
# Standalone SVG temperature and output chart, sampled raw data retained.
w,h=1100,620
pts=[(float(x['elapsed_s']),float(x['temperature_c']),float(x['output_permille'])/10) for x in rows]
end=max(x[0] for x in pts)
x=lambda t:65+t/end*990
y=lambda t:275-(t-20)/45*220
z=lambda u:540-u/100*210
parts=[f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}" viewBox="0 0 {w} {h}"><rect width="100%" height="100%" fill="white"/><g font-family="sans-serif" font-size="14"><text x="65" y="25">24 C ambient: room to 40 C, then 60 C</text>']
for v in [20,30,40,50,60]: parts.append(f'<path d="M65 {y(v)}H1055" stroke="#ddd"/><text x="20" y="{y(v)}">{v} C</text>')
for v in [0,25,50,75,100]: parts.append(f'<path d="M65 {z(v)}H1055" stroke="#ddd"/><text x="20" y="{z(v)}">{v}%</text>')
for col,axis,i in [('#db522c',y,1),('#246cba',z,2)]:
 points=' '.join(f'{x(p[0]):.1f},{axis(p[i]):.1f}' for p in pts)
 parts.append(f'<polyline points="{points}" fill="none" stroke="{col}" stroke-width="1.6"/>')
for sec in range(0,int(end)+1,120): parts.append(f'<text x="{x(sec)}" y="575">{sec}s</text>')
parts.append('</g></svg>')
(root/'curves.svg').write_text(''.join(parts))
