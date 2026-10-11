#!/usr/bin/env python3
"""Evaluate held-outs against frozen v1 without fitting, clipping or modifying it."""
import hashlib,importlib.util,json
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def load(name,file):
    spec=importlib.util.spec_from_file_location(name,Path(__file__).with_name(file));m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m);return m
summary=load('head_eval','evaluate-calibration-model.py');fit=load('local_fit','fit-local-calibration-model.py')
def main():
    frozen=ROOT/'diagnostics/local-piecewise-model-frozen-2026-10-11.json';original=frozen.read_bytes();model=json.loads(original)
    audit_path=ROOT/'diagnostics/motion-history-audit-2026-10-11.json';audit=json.loads(audit_path.read_text())
    history={r['source']:r for r in audit['timeline'] if r['kind']=='accepted_counted_jog'}
    results={}
    for joint in ('shoulder','elbow'):
        paths=sorted((ROOT/'recordings').glob(f'local-holdout6-{joint}-*.jsonl'))
        if len(paths)!=6:raise ValueError('Expected exactly six heldouts per joint')
        m=model['models'][joint];params=[m['knot_deg'],m['low_pulses_per_degree'],m['high_pulses_per_degree'],m['reversal_pulses']];lo,hi=m['training_envelope_deg'];rows=[]
        for path in paths:
            row=summary.summarize(path);h=history[row['source']]
            if h['history']=='unknown':raise ValueError('Unknown motion history cannot be assigned from filename')
            reversal=h['history']=='reversal'
            predicted=fit.predict(row['initial_angle_deg'],row['issued_pulses'],1 if row['signed_travel_deg']>0 else -1,reversal,params)
            error=predicted-row['final_angle_deg'];inside=lo<=row['initial_angle_deg']<=hi and lo<=row['final_angle_deg']<=hi
            row.update(reversal=reversal,history_evidence=h['previous_recorded_motion'],predicted_final_angle_deg=predicted,prediction_error_deg=error,within_training_envelope=inside,extrapolation=not inside,within_endpoint_tolerance=abs(error)<=.5,passed=abs(error)<=.5 and inside and row['quality_candidate']);rows.append(row)
        results[joint]=dict(trials=rows,all_pass=all(r['passed'] for r in rows),max_absolute_error_deg=max(abs(r['prediction_error_deg']) for r in rows),extrapolated_trials=[r['source'] for r in rows if r['extrapolation']])
    output=dict(artifact_type='frozen_v1_local_heldout_evaluation',globally_verified=False,installable=False,accepted_ratios=[],frozen_model_source=str(frozen.relative_to(ROOT)),frozen_model_sha256=hashlib.sha256(original).hexdigest(),history_audit_sha256=hashlib.sha256(audit_path.read_bytes()).hexdigest(),endpoint_tolerance_deg=.5,results=results,conclusion='Both frozen local models fail the predeclared endpoint tolerance. Do not install or claim whole-arm calibration.',history_correction='Shoulder calquiet forward01 was continuation, not reversal. Frozen v1 is preserved; its zero shoulder reversal coefficient makes existing predictions unchanged. No refit was used for these heldouts.',limitations=['First elbow heldout begins outside training bounds; flagged as extrapolation without clipping.','Passed telemetry quality does not establish predictive calibration accuracy.','All coordinates are reported encoder angles; independent physical-angle assessment remains separate.'])
    if frozen.read_bytes()!=original:raise RuntimeError('Frozen model changed during evaluation')
    target=ROOT/'diagnostics/local-heldout-evaluation-2026-10-11.json';target.write_text(json.dumps(output,indent=2)+'\n')
    print(json.dumps({j:dict(errors_deg=[r['prediction_error_deg'] for r in v['trials']],all_pass=v['all_pass'],extrapolated=v['extrapolated_trials']) for j,v in results.items()},indent=2))
if __name__=='__main__':main()
