#!/usr/bin/env python3
"""Offline diagnostic only: evaluate the frozen local head model, never install it."""
import hashlib,importlib.util,json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('recording_analysis',Path(__file__).with_name('analyze-recording.py'))
a=importlib.util.module_from_spec(spec);spec.loader.exec_module(a)
def summarize(path):
    events=a.read_events(path)
    if not events or events[-1].get('text')!='SHUTDOWN':raise ValueError(f'Incomplete recording: {path.name}')
    moves=a.analyze(events,path)['moves']
    if len(moves)!=1 or not moves[0]['accepted']:raise ValueError(f'Expected one accepted move: {path.name}')
    m=moves[0];j=m['joint']
    return dict(source=str(path.relative_to(ROOT)),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),joint=j,direction=m['direction'],initial_angle_deg=m['initial_angles_deg'][j],final_angle_deg=m['final_angles_deg'][j],signed_travel_deg=m['signed_delta_deg'][j],issued_pulses=m['issued_pulses'][j],endpoint_pulses_per_degree=m['pulses_per_degree'],measurement_mode=m['measurement_mode'],preparation_ms=m['preparation_ms'],quality_candidate=m['ratio_candidate'],quality_rejections=m['ratio_rejection_reasons'],interior_regression=m['interior_regression'].get(j),sleep_drift_deg=m['sleep_drift_deg'][j])
def model_trial(path):
    row=summarize(path)
    # These named four-stroke sequences were ordered F reversal, F continuation,
    # R reversal, R continuation. History is explicit, not inferred from fit.
    reversal=path.stem.endswith(('01-forward','03-reverse','forward-01','reverse-03'))
    predicted=(row['issued_pulses']-(260 if reversal else 0))/143.1
    error=predicted-abs(row['signed_travel_deg'])
    row.update(reversal=reversal,predicted_travel_deg=predicted,prediction_error_deg=error,within_frozen_half_degree_tolerance=abs(error)<=.5 and row['quality_candidate'])
    return row

def main():
    training=sorted((ROOT/'recordings').glob('head-adjacent-*.jsonl'))+sorted((ROOT/'recordings').glob('calquiet-head-*.jsonl'))
    heldout=sorted((ROOT/'recordings').glob('head-holdout6-*.jsonl'))
    if len(training)!=8 or len(heldout)!=4:raise ValueError('Expected eight training and four held-out recordings')
    train=[model_trial(p) for p in training];test=[model_trial(p) for p in heldout]
    other={}
    for j in ('shoulder','elbow'):
        paths=sorted((ROOT/'recordings').glob(f'calquiet-{j}-*.jsonl'))+sorted((ROOT/'recordings').glob(f'calrepeat-{j}-*.jsonl'))
        other[j]=[summarize(p) for p in paths]
    endpoints=[v for row in train+test for v in (row['initial_angle_deg'],row['final_angle_deg'])]
    report=dict(artifact_type='offline_calibration_model_diagnostic',globally_verified=False,installable=False,accepted_ratios=[],coordinate_system='Reported calibrated AS5600 angles; independent physical-angle accuracy not established',head_model=dict(formula='issued_pulses = 143.1 * abs(reported_angle_change_deg) + 260 * reversal',gain_pulses_per_degree=143.1,reversal_pulses=260,endpoint_tolerance_deg=.5,tolerance_frozen_before_heldout=True,sampled_angle_envelope_deg=[min(endpoints),max(endpoints)],heldout_angle_envelope_deg=[min(v for row in test for v in (row['initial_angle_deg'],row['final_angle_deg'])),max(v for row in test for v in (row['initial_angle_deg'],row['final_angle_deg']))],scope='Local single-joint endpoint prediction, observed rates and load only; envelope is not certification of every pose',training=train,heldout=test,all_heldout_pass=all(t['within_frozen_half_degree_tolerance'] for t in test),max_heldout_abs_error_deg=max(abs(t['prediction_error_deg']) for t in test)),other_joint_trials=other,assessments=['Head reversal history explains much of endpoint conversion variation; interior trajectory accuracy is not validated by endpoint fit.','Shoulder upper region approximately -7 to +4 degrees repeats near171–174 pulses/degree. Lower approximately -18 to -7 repeats near121–126, with an earlier111.55 outlier retained. No full-range constant conversion or lookup is accepted.','Elbow lower region approximately50–61 degrees repeats near148–152 pulses/degree; upper approximately61–71 repeats near131–139. Direction/history remains a confounder. Independent held-out lookup validation is absent.','CAD branch models constant 16:1 cycloidal drives; no independent encoder magnet mounting geometry was found. A nonlinear lookup can predict encoder readings without establishing true physical angle.','This artifact is deliberately not the ratio loader schema. No values are installed or marked globally verified.'])
    target=ROOT/'diagnostics/calibration-model-evaluation-2026-10-11.json'
    target.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(output=str(target),head_heldout_pass=report['head_model']['all_heldout_pass'],maximum_heldout_error_deg=report['head_model']['max_heldout_abs_error_deg'],other_joint_trial_counts={j:len(rows) for j,rows in other.items()}),indent=2))
if __name__=='__main__':main()
