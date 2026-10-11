#!/usr/bin/env python3
"""Offline fit on explicit eight-trial training sets; never reads held-out files."""
import json,hashlib
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def solve(matrix,vector):
    n=len(vector);a=[list(row)+[y] for row,y in zip(matrix,vector)]
    for i in range(n):
        pivot=max(range(i,n),key=lambda j:abs(a[j][i]));a[i],a[pivot]=a[pivot],a[i]
        v=a[i][i];a[i]=[x/v for x in a[i]]
        for j in range(n):
            if j!=i:
                v=a[j][i];a[j]=[x-v*y for x,y in zip(a[j],a[i])]
    return [row[-1] for row in a]
def coordinate(q,k,low,high):return (q-k)*(low if q<k else high)
def predict(initial,pulses,direction_sign,reversal,p):
    k,low,high,b=p
    c=coordinate(initial,k,low,high)+direction_sign*(pulses-b*reversal)
    return k+c/(low if c<0 else high)
def main():
    # Frozen v1 contains a subsequently corrected shoulder history label.
    # Never overwrite it or refit it after held-out data has been observed.
    if (ROOT/'diagnostics/local-piecewise-model-frozen-2026-10-11.json').exists():
        raise SystemExit('Frozen v1 already exists; preserved. See motion-history-audit-2026-10-11.json for the history correction. Any refit requires a new explicitly post-holdout artifact.')
    report=json.loads((ROOT/'diagnostics/calibration-model-evaluation-2026-10-11.json').read_text())
    models={}
    for joint,k in [('shoulder',-7.),('elbow',61.)]:
        rows=report['other_joint_trials'][joint]
        if len(rows)!=8:raise ValueError('Exactly eight pre-heldout training rows required')
        X=[];Y=[]
        for t in rows:
            p=ROOT/t['source']
            if hashlib.sha256(p.read_bytes()).hexdigest()!=t['sha256']:raise ValueError('Training source changed')
            lo,hi=sorted([t['initial_angle_deg'],t['final_angle_deg']]);low=max(0,min(hi,k)-lo);high=hi-lo-low
            rev=t['source'].endswith(('forward-01.jsonl','reverse-03.jsonl'))
            X.append([low,high,int(rev)]);Y.append(t['issued_pulses'])
        def fit(cols):return solve([[sum(x[i]*x[j] for x in X) for j in range(cols)] for i in range(cols)],[sum(x[i]*y for x,y in zip(X,Y)) for i in range(cols)])
        gains=fit(3)
        if gains[2]<0:gains=fit(2)+[0.]
        p=[k]+gains;trials=[]
        for t,x in zip(rows,X):
            predicted=predict(t['initial_angle_deg'],t['issued_pulses'],1 if t['signed_travel_deg']>0 else -1,x[2],p)
            trials.append(dict(t,reversal=bool(x[2]),predicted_final_angle_deg=predicted,prediction_error_deg=predicted-t['final_angle_deg']))
        endpoints=[v for t in rows for v in (t['initial_angle_deg'],t['final_angle_deg'])]
        models[joint]=dict(knot_deg=k,low_pulses_per_degree=gains[0],high_pulses_per_degree=gains[1],reversal_pulses=gains[2],training_envelope_deg=[min(endpoints),max(endpoints)],max_training_absolute_error_deg=max(abs(t['prediction_error_deg']) for t in trials),training_trials=trials)
    result=dict(artifact_type='frozen_local_piecewise_calibration_diagnostic',globally_verified=False,installable=False,accepted_ratios=[],models=models,model_definition='N = abs(F(final)-F(initial)) + reversal_pulses*reversal; F(q)=(q-knot)*gain_below_or_above_knot',fit_policy='All eight pre-heldout trials per joint, least squares on pulse counts, nonnegative reversal offset; knots fixed at previously observed region boundaries, not optimized',validation_plan=dict(endpoint_tolerance_deg=.5,desired_travel_deg=6,required_conditions=['Every heldout passes existing telemetry/settling guards','Both directions and known reversal/continuation history','Sample both sides and cross the knot','Never extrapolate outside training envelope','No parameter adjustment after seeing heldouts'],historical_failure_policy='Any pre-existing training error above tolerance remains disclosed; passing new trials does not erase it'),limitations=['Reported encoder coordinates only; mounting confirmation is not independent physical angle validation.','Local endpoint prediction only, not coordinated trajectory/path accuracy.','Shoulder retains an earlier lower-region training mismatch above0.5degree; a constant-gain or globallyverified calibration is not justified.'])
    target=ROOT/'diagnostics/local-piecewise-model-frozen-2026-10-11.json'
    target.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({j:{k:v for k,v in model.items() if k!='training_trials'} for j,model in models.items()},indent=2))
if __name__=='__main__':main()
