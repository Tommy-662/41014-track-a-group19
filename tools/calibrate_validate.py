"""Held-out rendered poses validate analytic simulation intrinsics/extrinsics.
Ground truth is used to position the test rig and calculate validation errors only.
No navigation or controller runs in this test.
"""
import sys,json,csv
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from run_sim import *
def main():
    d=Demo(); rows=[]
    try:
        poses=[(x,y,a) for x,y in [(-.2,.2),(.2,.4),(.6,.6),(1.,1.),(0.,1.2)] for a in [0.,.6,1.4,2.4,-2.4]]
        poses += [(-.3,y,np.pi) for y in [-.08,0,.08]]
        for x,y,a in poses:
            mujoco.mj_resetData(d.m,d.d); d.d.qpos[:2]=[x,y]; d.d.qpos[3:7]=[np.cos(a/2),0,0,np.sin(a/2)]; mujoco.mj_forward(d.m,d.d)
            z,dock=d.sensors.camera()
            rows.append(dict(x=x,y=y,yaw=a,detected=z is not None,position_error_m=None if z is None else float(np.linalg.norm(z[:2]-[x,y])),yaw_error_rad=None if z is None else float(abs(wrap(z[2]-a))),station_relative_error_m=None if dock is None else float(np.linalg.norm(dock-(np.array([[np.cos(a),np.sin(a)],[-np.sin(a),np.cos(a)]])@(np.array([-.698,0])-np.array([x,y])))))))
        result=dict(method='Analytic pinhole intrinsics and XML rigid extrinsics; held-out pose validation, not hardware calibration or fitted checkerboard calibration',poses=rows,reprojection_mean_px=float(np.mean(d.sensors.reprojection)))
        for key in ['position_error_m','yaw_error_rad','station_relative_error_m']:
            values=[r[key] for r in rows if r[key] is not None]
            result[key+'_rmse']=float(np.sqrt(np.mean(np.square(values)))) if values else None
            result[key+'_max']=max(values,default=None)
        out=ROOT/'results'/'calibration_validation.json'; out.write_text(json.dumps(result,indent=2)); print(json.dumps({k:v for k,v in result.items() if k!='poses'},indent=2))
    finally: d.sensors.close()
if __name__=='__main__': main()
