#!/usr/bin/env python3
"""Read-only recording history reconstruction; preserves the original frozen model."""
import importlib.util,json,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sp=importlib.util.spec_from_file_location('analysis',Path(__file__).with_name('analyze-recording.py'));a=importlib.util.module_from_spec(sp);sp.loader.exec_module(a)
def main():
    timeline=[];sources=[];issues=[]
    for path in sorted((ROOT/'recordings').glob('*.jsonl'),key=lambda p:p.stat().st_birthtime):
        try:events=[json.loads(line) for line in path.read_text().splitlines() if line.strip()]
        except Exception as exc:issues.append(dict(source=path.name,error=str(exc)));continue
        source=str(path.relative_to(ROOT));birth=path.stat().st_birthtime
        sources.append(dict(source=source,sha256=hashlib.sha256(path.read_bytes()).hexdigest(),filesystem_birth_unix_s=birth))
        def append(event,joint,direction,kind,board_ms=None,**extra):
            timeline.append(dict(source=source,event_time_s=event.get('time',0),ordering_time_s=birth+event.get('time',0),board_ms=board_ms,joint=joint,direction=direction,kind=kind,**extra))
        # Counted sessions have explicit accepted motion IDs plus positive final counts.
        if events and events[0].get('version')==1:
            try:
                report=a.analyze(events)
                for m in report['moves']:
                    if not m.get('accepted'):continue
                    if not m.get('joint'):
                        issues.append(dict(source=source,error='Accepted non-JOG motion requires manual history interpretation'));continue
                    joint=m['joint']
                    if m['issued_pulses'][joint]<=0:continue
                    event=next(e for e in events if e.get('type')=='command' and e.get('time')==m['command_time_s'])
                    sample=next(e['sample'] for e in events if e.get('type')=='sample' and e['sample']['owner']==m['owner'] and e['sample']['id']==m['id'])
                    append(event,joint,m['direction'],'accepted_counted_jog',sample['ms'],owner=m['owner'],motion_id=m['id'],issued_pulses=m['issued_pulses'][joint])
            except Exception as exc:issues.append(dict(source=source,error=str(exc)))
        # PWM records directly report the active physical direction. Reject command-only evidence.
        prior=None
        for event in events:
            if event.get('type')!='motor_sample':continue
            s=event['sample'];key=(s.get('axis'),s.get('dir')) if s.get('run') and s.get('hz',0)>0 and not s.get('error') else None
            if key is not None and key!=prior:
                feedback=min((e for e in events if e.get('sample',{}).get('ms') is not None),key=lambda e:abs(e.get('time',0)-event.get('time',0)),default=None)
                append(event,a.JOINTS[a.MOTORS.index(key[0])],key[1],'observed_active_manual_pwm',feedback['sample']['ms'] if feedback else None)
            prior=key
        # Passive GUI recording has no commanded direction: observed motion invalidates any assumed command history.
        previous=None;marked=set()
        for event in events:
            if 'joints' not in event:continue
            if previous:
                for j in a.JOINTS:
                    x=event['joints'][j].get('angle_deg');y=previous['joints'][j].get('angle_deg')
                    if x is not None and y is not None and abs(x-y)>.3 and j not in marked:
                        append(event,j,None,'passive_motion_direction_unknown',event.get('board_ms'));marked.add(j)
            previous=event
    timeline.sort(key=lambda row:row['ordering_time_s'])
    previous={};dedup=set()
    for row in timeline:
        j=row['joint'];prior=previous.get(j)
        row['previous_recorded_motion']=dict(source=prior['source'],direction=prior['direction'],board_ms=prior['board_ms']) if prior else None
        row['history']='unknown' if prior is None or prior['direction'] is None or row['direction'] is None else ('reversal' if prior['direction']!=row['direction'] else 'continuation')
        row['same_boot_order_confirmed']=bool(prior and row['board_ms'] is not None and prior['board_ms'] is not None and row['board_ms']>=prior['board_ms'])
        previous[j]=row
    by_source={row['source']:row for row in timeline if row['kind']=='accepted_counted_jog'}
    frozen=ROOT/'diagnostics/local-piecewise-model-frozen-2026-10-11.json'
    original=json.loads(frozen.read_text());corrections=[]
    for joint,model in original['models'].items():
        for trial in model['training_trials']:
            row=by_source.get(trial['source'])
            if row and row['history']!='unknown' and trial['reversal']!=(row['history']=='reversal'):
                corrections.append(dict(source=trial['source'],original_reversal=trial['reversal'],corrected_history=row['history'],prior_motion=row['previous_recorded_motion'],impact='Original artifact preserved. Shoulder frozen reversal coefficient is zero, so its original numerical predictions are unchanged. Any refit must be labeled a new model after held-out data.'))
    result=dict(artifact_type='motion_history_provenance_correction',original_frozen_model_sha256=hashlib.sha256(frozen.read_bytes()).hexdigest(),frozen_v1_modified=False,corrections=corrections,timeline=timeline,recording_sources=sources,issues=issues,ordering_method='Filesystem recording-creation timestamp anchors each relative event timestamp; recorded MCU milliseconds accompany each event and corroborate order where monotonic. MCU timestamps reset on reboot. Filenames never determine direction history.',limitations=['Recordings omit absolute wall-clock event timestamps and boot IDs. Filesystem creation metadata supplies cross-boot order and is explicitly retained; it is weaker evidence than logged absolute timestamps.','No recording audit proves absence of unrecorded physical intervention or manually moved joints. Unknown passive motion invalidates assumed command history until another observed commanded movement.','Direction after zero issued pulses is not treated as physical direction history.'])
    target=ROOT/'diagnostics/motion-history-audit-2026-10-11.json';target.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(output=str(target),motions=len(timeline),corrections=corrections,issues=issues),indent=2))
if __name__=='__main__':main()
