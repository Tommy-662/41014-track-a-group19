"""Run the declared trial matrix. Every trial preserves stdout, CSV and JSON.
Usage: python tools/run_experiments.py [--workers 2] [--skip nominal_s7]
"""
import argparse, concurrent.futures, json, subprocess, sys, time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--workers',type=int,default=2); ap.add_argument('--skip',nargs='*',default=[]); a=ap.parse_args()
    cases={'nominal':[], 'dropout40':['--dropout','.4'], 'blackout20to26':['--blackout','20','26'], 'encoder_plus2pct':['--encoder-scale','1.02']}
    jobs=[(f'{name}_s{seed}',seed,args,240) for name,args in cases.items() for seed in (7,11,23)]
    jobs.append(('vision_loss100_s7',7,['--dropout','1'],50))
    jobs=[j for j in jobs if j[0] not in a.skip]
    (ROOT/'results').mkdir(exist_ok=True)
    def run(job):
        name,seed,args,seconds=job
        cmd=[sys.executable,str(ROOT/'run_sim.py'),'--headless','--seed',str(seed),'--seconds',str(seconds),'--output',f'results/{name}',*args]
        if name in ('nominal_s7','dropout40_s7'): cmd+=['--video']
        start=time.time()
        with (ROOT/'results'/f'{name}.stdout.txt').open('w',encoding='utf-8') as log:
            proc=subprocess.run(cmd,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,timeout=900)
        record=dict(name=name,command=cmd,returncode=proc.returncode,wall_seconds=time.time()-start)
        print(json.dumps(record),flush=True); return record
    with concurrent.futures.ThreadPoolExecutor(max_workers=a.workers) as pool:
        records=list(pool.map(run,jobs))
    (ROOT/'results'/'experiment_execution.json').write_text(json.dumps(records,indent=2),encoding='utf-8')
    if any(r['returncode']!=0 for r in records if not r['name'].startswith('vision_loss')): raise SystemExit(1)
if __name__=='__main__': main()
