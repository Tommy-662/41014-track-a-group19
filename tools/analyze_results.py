"""Generate auditable summary tables and scientific plots from raw logs."""
import sys,json,csv,math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT=Path(__file__).resolve().parents[1]; sys.path.insert(0,str(ROOT))
from run_sim import FeedbackController
RESULTS=ROOT/'results'
def load(name):
    return np.genfromtxt(RESULTS/f'{name}.csv',delimiter=',',names=True,dtype=None,encoding='utf-8')
def success(r):
    return r['stage']=='CHARGING' and r['waypoints']==r['waypoint_count'] and r['obstacle_contacts']==0 and r['charge_seconds']>=r['charge_demo_seconds']-1e-9 and r['battery_current']>0 and r['charged_ah']>0 and r['dock_truth_error_m']<=.09 and r['dock_truth_heading_error_rad']<=.12
def main():
    conditions=['nominal','dropout40','blackout20to26','encoder_plus2pct']; summary=[]; records=[]
    keys=['rmse_m','yaw_rmse_rad','odometry_rmse_m','path_tracking_rmse_m','dock_truth_error_m','dock_truth_heading_error_rad','time','mean_nees','effort_torque_squared_integral']
    for condition in conditions:
        runs=[json.loads((RESULTS/f'{condition}_s{s}.json').read_text()) for s in [7,11,23]]
        n=len(runs); wins=sum(success(r) for r in runs); z=1.96; ph=wins/n; den=1+z*z/n; cen=(ph+z*z/(2*n))/den; half=z*math.sqrt(ph*(1-ph)/n+z*z/(4*n*n))/den
        row=dict(condition=condition,n=n,successes=wins,success_wilson95=[cen-half,cen+half],contacts=sum(r['obstacle_contacts'] for r in runs))
        for key in keys:
            vals=np.array([r[key] for r in runs]); row[key]=dict(mean=float(vals.mean()),sd=float(vals.std(ddof=1)),min=float(vals.min()),max=float(vals.max()))
        summary.append(row)
        for r in runs: records.append(dict(condition=condition,seed=r['seed'],success=success(r),**{k:r[k] for k in keys}))
    (RESULTS/'summary.json').write_text(json.dumps(summary,indent=2))
    with (RESULTS/'trials.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(records[0])); w.writeheader(); w.writerows(records)
    c=FeedbackController(); (RESULTS/'model_analysis.json').write_text(json.dumps(dict(A=c.A.tolist(),B=c.B.tolist(),K=c.K.tolist(),poles=c.eigenvalues.tolist(),controllability_rank=int(np.linalg.matrix_rank(np.hstack([c.B,c.A@c.B,c.A@c.A@c.B]))),observation_rank=3,riccati_residual=float(np.max(abs(c.A.T@c.P@c.A-c.A.T@c.P@c.B@c.K+c.Q-c.P)))),indent=2))
    plt.rcParams.update({'font.size':10,'axes.grid':True,'grid.alpha':.2,'figure.dpi':150})
    fig,axes=plt.subplots(2,2,figsize=(12,9),layout='constrained')
    for condition,color in zip(conditions,['#136f9b','#e08220','#8b4aaa','#22865b']):
        d=load(condition+'_s7'); t=d['time']; truth=np.c_[d['truth_x_evaluation_only'],d['truth_y_evaluation_only']]; ekf=np.c_[d['ekf_x'],d['ekf_y']]
        axes[0,0].plot(truth[:,0],truth[:,1],color=color,label=condition)
        axes[0,1].plot(t,np.linalg.norm(ekf-truth,axis=1)*1000,color=color,label=condition)
        e=np.genfromtxt(RESULTS/f'{condition}_s7_evaluation.csv',delimiter=',',names=True)
        axes[1,1].plot(e['time'],e['nees'],color=color,alpha=.7,label=condition)
    axes[0,0].add_patch(plt.Rectangle((.4,-.12),.24,.24,color='#e08220',alpha=.35)); goals=np.array(json.loads((RESULTS/'nominal_s7.json').read_text())['waypoint_goals']); axes[0,0].scatter(*goals.T,marker='x',color='black',label='Waypoints'); axes[0,0].set(aspect='equal',title='Evaluation trajectories (seed 7)',xlabel='World x [m]',ylabel='World y [m]'); axes[0,0].legend(fontsize=8)
    axes[0,1].axvspan(20,26,color='#8b4aaa',alpha=.1); axes[0,1].set(title='Time-aligned EKF position error',xlabel='Time [s]',ylabel='Error [mm]')
    d=load('blackout20to26_s7'); ax=axes[1,0]; error=d['ekf_x']-d['truth_x_evaluation_only']; ax.plot(d['time'],error,label='x estimate - truth',color='#136f9b'); ax.fill_between(d['time'],-2*d['sigma_x'],2*d['sigma_x'],alpha=.2,label='+/-2 sigma x'); ax.axvspan(20,26,color='#8b4aaa',alpha=.12,label='Complete vision blackout'); ax.set(title='Estimate uncertainty under blackout',xlabel='Time [s]',ylabel='x error [m]'); ax.legend(fontsize=8)
    axes[1,1].axhline(3,color='black',ls='--',lw=1,label='Ideal mean 3'); axes[1,1].set(title='3-state NEES (correlated samples)',xlabel='Time [s]',ylabel='NEES'); axes[1,1].legend(fontsize=7)
    fig.savefig(RESULTS/'evaluation.png'); plt.close(fig)
    fig,axes=plt.subplots(1,3,figsize=(13,4),layout='constrained')
    for ax,key,label,scale in zip(axes,['rmse_m','path_tracking_rmse_m','dock_truth_error_m'],['Localization RMSE [mm]','Current-path RMSE [mm]','Final docking error [mm]'],[1000]*3):
        ax.bar(range(4),[s[key]['mean']*scale for s in summary],yerr=[s[key]['sd']*scale for s in summary],capsize=4,color=['#136f9b','#e08220','#8b4aaa','#22865b']); ax.set_xticks(range(4),['Nominal','40% loss','6 s blackout','Encoder +2%'],rotation=18); ax.set_ylabel(label); ax.set_title('3 seeds; bars mean, whiskers sample SD')
    fig.savefig(RESULTS/'comparison.png'); plt.close(fig)
    d=load('blackout20to26_s7'); fig,ax=plt.subplots(figsize=(11,4.7),layout='constrained')
    ax.plot(d['time'],d['ekf_x']-d['truth_x_evaluation_only'],label='Estimated x - true x',color='#136f9b',lw=2)
    ax.fill_between(d['time'],-2*d['sigma_x'],2*d['sigma_x'],alpha=.22,label='+/-2 sigma x')
    ax.axvspan(20,26,color='#8b4aaa',alpha=.15,label='Complete vision blackout')
    ax.set(title='Seed 7: uncertainty grows without camera corrections',xlabel='Time [s]',ylabel='Position error and uncertainty [m]'); ax.legend(loc='lower right')
    fig.savefig(RESULTS/'uncertainty_video.png'); plt.close(fig)
    lines=['# Repeated-trial results','', 'Fresh runs from the submitted source; 3 seeds per condition. Mean +/- sample SD. Completion time includes a one-second charging demonstration.','', '| Condition | success | localization RMSE mm | path RMSE mm | dock error mm | time s | mean NEES |','|---|---:|---:|---:|---:|---:|---:|']
    def val(row,key,scale=1): return f"{row[key]['mean']*scale:.2f} +/- {row[key]['sd']*scale:.2f}"
    for row in summary: lines.append(f"| {row['condition']} | {row['successes']}/3 | {val(row,'rmse_m',1000)} | {val(row,'path_tracking_rmse_m',1000)} | {val(row,'dock_truth_error_m',1000)} | {val(row,'time')} | {val(row,'mean_nees')} |")
    lines+=['','Each 3/3 rate has a 95% Wilson interval approximately [0.44,1.00]; trials are few and share one scene. Zero contact samples were observed in successful trials. Path RMSE uses the changing planned polyline, so it is not a timed trajectory metric. Localization RMSE is not final docking error.','', '![Trajectory, error, uncertainty and NEES](../results/evaluation.png)','', '![Repeated-trial comparison](../results/comparison.png)','', '## Trial detail','', '| Condition | seed | odom RMSE m | yaw RMSE rad | dock yaw rad | torque effort N^2 m^2 s |','|---|---:|---:|---:|---:|---:|']
    for r in records: lines.append(f"| {r['condition']} | {r['seed']} | {r['odometry_rmse_m']:.4f} | {r['yaw_rmse_rad']:.4f} | {r['dock_truth_heading_error_rad']:.4f} | {r['effort_torque_squared_integral']:.4f} |")
    cal=json.loads((RESULTS/'calibration_validation.json').read_text()); detected=sum(r['detected'] for r in cal['poses']); station=[r for r in cal['poses'] if r['station_relative_error_m'] is not None]; near=[r['station_relative_error_m'] for r in station if r['x']==-.3]
    lines+=['','## Sensor validation','',f"28 held-out poses: {detected}/28 base-pose detections; {len(station)}/28 station-relative detections. Base position RMSE {cal['position_error_m_rmse']*1000:.3f} mm; yaw RMSE {cal['yaw_error_rad_rmse']:.6f} rad. Station-relative RMSE over detected cases {cal['station_relative_error_m_rmse']*1000:.2f} mm; max {cal['station_relative_error_m_max']*1000:.2f} mm. The three near-dock front-facing samples have station-relative RMSE {np.sqrt(np.mean(np.square(near)))*1000:.2f} mm. Long-range oblique planar PnP depth is less constrained despite low pixel residual. Missing station detections are expected when facing away and are retained in raw results."]
    fail_path=RESULTS/'vision_loss100_s7.json'
    if fail_path.exists():
        f=json.loads(fail_path.read_text()); lines+=['','## Failure boundary','',f"100% camera observation loss, seed 7, 50 s evaluation budget: stage {f['stage']}, estimated waypoint counter {f['waypoints']}/{f['waypoint_count']} (not independently verified true arrivals), position RMSE {f['rmse_m']:.3f} m, {f['obstacle_contacts']} contact samples. The run is a failed mission and is not included in the 12-trial success denominator. Without camera measurements, no visual station estimate is available, so the visual-docking requirement cannot be completed; odometry also accumulates slip error. This run does not claim a 240 s timeout test."]
    rpaths=[RESULTS/'r01_s7.json',RESULTS/'r10_s7.json']
    if all(p.exists() for p in rpaths):
        lines+=['','## EKF measurement-noise sensitivity (one seed, exploratory)','', '| R multiplier | localization RMSE mm | mean NEES | path RMSE mm | torque effort | success |','|---|---:|---:|---:|---:|---|']
        for name in ['r01_s7','nominal_s7','r10_s7']:
            r=json.loads((RESULTS/f'{name}.json').read_text()); lines.append(f"| {r['r_scale']} | {r['rmse_m']*1000:.3f} | {r['mean_nees']:.3f} | {r['path_tracking_rmse_m']*1000:.3f} | {r['effort_torque_squared_integral']:.3f} | {success(r)} |")
        lines+=['','The table compares measurement weighting, actual error and reported uncertainty on one seed. It is a sensitivity probe, not a statistically established optimum. The common baseline is retained across the repeated trials. Smaller R gives camera observations more weight; the resulting NEES depends on both actual error and posterior covariance.']
    lines+=['','For the interpretation of these trials and the limitations of the metrics, see [Discussion](discussion.md).']
    (ROOT/'docs'/'results.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    print(json.dumps(summary,indent=2))
if __name__=='__main__': main()
