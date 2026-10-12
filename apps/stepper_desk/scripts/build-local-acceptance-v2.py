#!/usr/bin/env python3
"""Versioned retrospective endpoint acceptance; no installation or hardware access."""
import hashlib,json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
TARGET=ROOT/'diagnostics/local-endpoint-acceptance-v2-2026-10-11.json'
def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def conservative_region(training_bounds,rows):
    eligible=[r for r in rows if r['quality_candidate'] and not r.get('extrapolation',False)]
    spans=[]
    for direction in (0,1):
        points=[v for r in eligible if r['direction']==direction for v in (r['initial_angle_deg'],r['final_angle_deg'])]
        if not points:raise ValueError('Both directions need non-extrapolated evidence')
        spans.append((min(points),max(points)))
    bounds=[max(training_bounds[0],*(x[0] for x in spans)),min(training_bounds[1],*(x[1] for x in spans))]
    if bounds[0]>=bounds[1]:raise ValueError('No common local region')
    return bounds

def build():
    names=['calibration-model-evaluation-2026-10-11.json','local-piecewise-model-frozen-2026-10-11.json','local-heldout-evaluation-2026-10-11.json','motion-history-audit-2026-10-11.json']
    evidence={name:json.loads((ROOT/'diagnostics'/name).read_text()) for name in names}
    head=evidence[names[0]]['head_model'];frozen=evidence[names[1]];held=evidence[names[2]];audit=evidence[names[3]]
    history={r['source']:r for r in audit['timeline'] if r['kind']=='accepted_counted_jog'}
    if held['frozen_model_sha256']!=digest(ROOT/'diagnostics'/names[1]):raise ValueError('Frozen model hash mismatch')
    joints={};sources={}
    for joint in ('head','shoulder','elbow'):
        if joint=='head':
            train=head['training'];test=head['heldout'];bounds=head['sampled_angle_envelope_deg'];params=dict(kind='gain_plus_reversal',gain_pulses_per_degree=143.1,reversal_pulses=260)
        else:
            m=frozen['models'][joint];train=m['training_trials'];test=held['results'][joint]['trials'];bounds=m['training_envelope_deg']
            params={k:m[k] for k in ('knot_deg','low_pulses_per_degree','high_pulses_per_degree','reversal_pulses')};params['kind']='piecewise_cumulative_gain_plus_reversal'
        rows=[]
        for role,items in [('training',train),('heldout',test)]:
            for item in items:
                path=ROOT/item['source']
                if digest(path)!=item['sha256']:raise ValueError('Recording hash mismatch')
                sources[item['source']]=item['sha256']
                h=history[item['source']]
                row=dict(item,evidence_role=role,corrected_recorded_history=h['history'],prior_motion_evidence=h['previous_recorded_motion'],within_retrospective_1_5_deg=abs(item['prediction_error_deg'])<=1.5)
                row['scope_eligible']=row['within_retrospective_1_5_deg'] and item['quality_candidate'] and not item.get('extrapolation',False)
                rows.append(row)
        if not all(row['within_retrospective_1_5_deg'] and row['quality_candidate'] for row in rows):raise ValueError('Observed endpoint/quality threshold failed')
        joints[joint]=dict(model=params,proposed_local_operating_region_deg=conservative_region(bounds,test),region_definition='Intersection of training bounds and both-direction heldout envelopes; extrapolated trials excluded. This is a sampled local region, not certification of every intermediate pose.',observed_max_absolute_endpoint_error_deg=max(abs(row['prediction_error_deg']) for row in rows),remaining_margin_to_1_5_deg=1.5-max(abs(row['prediction_error_deg']) for row in rows),training_envelope_deg=bounds,trial_summaries=rows,excluded_from_scope=[row['source'] for row in rows if not row['scope_eligible']])
    return dict(artifact_type='retrospective_local_endpoint_acceptance',acceptance_version=2,date='2026-10-11',globally_verified=False,installable=False,accepted_ratios=[],data_accepted_for_local_endpoint_model_evaluation=True,tolerance_deg=1.5,tolerance_basis='Retrospective user acceptance of the already-observed shoulder1.41degree and elbow1.16degree endpoint errors. This replaces the acceptance criterion for local operational planning only; it is not a predeclared test success.',original_half_degree_failures_preserved=True,parameter_refit_performed=False,user_mounting_confirmation='User confirmed magnets centered/coaxial and rigidly attached to output shafts.',independent_physical_angle_accuracy_verified=False,physical_microstep_settings_verified=False,joints=joints,evidence_artifacts=[dict(source='diagnostics/'+name,sha256=digest(ROOT/'diagnostics'/name)) for name in names],recording_sources=[dict(source=name,sha256=sha) for name,sha in sorted(sources.items())],limitations=['No full mechanical-range or coordinated-path calibration is claimed.','Observed endpoint errors are not guaranteed upper bounds for future moves; payload, speed, direction history, sleep drift and joint coupling can change behavior.','The first elbow heldout began outside frozen training bounds; its endpoint error is reported but it is excluded from defining supported scope, without clipping or silent waiver.','The original frozen shoulder history label was corrected in the audit. Frozen parameters and original reports remain unchanged.','Physical microstep settings remain unverified; empirical pulse-to-reported-angle models do not certify motor-shaft RPM.','No firmware limit, stale-sensor guard, watchdog, sleep behavior or calibration loader state is changed.'])
def validate(data):
    if data.get('artifact_type')!='retrospective_local_endpoint_acceptance' or data.get('acceptance_version')!=2:raise ValueError('Wrong diagnostic schema')
    if data.get('globally_verified') is not False or data.get('installable') is not False or data.get('accepted_ratios')!=[]:raise ValueError('Scope must remain local and noninstallable')
    if data.get('tolerance_deg')!=1.5 or data.get('parameter_refit_performed') is not False:raise ValueError('Unexpected criterion or refit')
    for item in data['evidence_artifacts']+data['recording_sources']:
        if digest(ROOT/item['source'])!=item['sha256']:raise ValueError('Evidence hash mismatch')
    for j in data['joints'].values():
        actual=max(abs(r['prediction_error_deg']) for r in j['trial_summaries'])
        if actual>1.5 or actual!=j['observed_max_absolute_endpoint_error_deg']:raise ValueError('Residual summary mismatch')
        region=conservative_region(j['training_envelope_deg'],[r for r in j['trial_summaries'] if r['evidence_role']=='heldout'])
        if region!=j['proposed_local_operating_region_deg']:raise ValueError('Unsupported region')
    return True
if __name__=='__main__':
    data=build();validate(data)
    with TARGET.open('x') as stream:json.dump(data,stream,indent=2);stream.write('\n')
    print(json.dumps({j:dict(region=v['proposed_local_operating_region_deg'],max_error=v['observed_max_absolute_endpoint_error_deg'],excluded=v['excluded_from_scope']) for j,v in data['joints'].items()},indent=2))
