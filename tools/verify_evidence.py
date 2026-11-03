"""Check evidence provenance and metric criteria without rerunning physics."""
import csv,hashlib,json,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def main():
    records=[]; errors=[]
    for condition in ['nominal','dropout40','blackout20to26','encoder_plus2pct']:
        for seed in [7,11,23]:
            name=f'{condition}_s{seed}'; r=json.loads((ROOT/'results'/f'{name}.json').read_text())
            for path,expected in r['source_sha256'].items():
                if hashlib.sha256((ROOT/path).read_bytes()).hexdigest()!=expected: errors.append(f'{name}: source changed: {path}')
            ok=r['stage']=='CHARGING' and r['waypoints']==r['waypoint_count'] and r['obstacle_contacts']==0 and r['charge_seconds']>=r['charge_demo_seconds']-1e-9 and r['battery_current']>0 and r['charged_ah']>0 and r['dock_truth_error_m']<=.09 and r['dock_truth_heading_error_rad']<=.12
            if not ok: errors.append(f'{name}: failed independent criteria')
            rows=list(csv.DictReader((ROOT/'results'/f'{name}.csv').open()))
            extra=list(csv.DictReader((ROOT/'results'/f'{name}_evaluation.csv').open()))
            if len(rows)!=len(extra) or any(a['time']!=b['time'] for a,b in zip(rows,extra)): errors.append(f'{name}: evaluation timestamps misaligned')
            records.append(dict(name=name,success=ok,samples=len(rows),timestamp_utc=r['timestamp_utc']))
    failure=json.loads((ROOT/'results/vision_loss100_s7.json').read_text()); assert failure['stage']!='CHARGING'
    fresh=json.loads((ROOT/'results/fresh_py312_s7.json').read_text()); original=json.loads((ROOT/'results/nominal_s7.json').read_text())
    for k in ['stage','waypoints','rmse_m','dock_truth_error_m']:
        if fresh[k]!=original[k]: errors.append(f'fresh environment differed: {k}')
    result=dict(repeated_trials=records,fresh_environment_python=fresh['python'],fresh_nominal_matches_exactly=not any('fresh environment' in e for e in errors),errors=errors)
    (ROOT/'results/evidence_verification.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2)); return bool(errors)
if __name__=='__main__': raise SystemExit(main())
