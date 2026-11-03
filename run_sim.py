"""Track A demo: simulated sensors -> EKF -> feedback -> physical wheels.
Ground truth is isolated in SensorSimulator and Evaluator, never controller inputs.
"""
import argparse, csv, heapq, json, math, time, hashlib, platform
from datetime import datetime, timezone
from pathlib import Path
import cv2
import mujoco
import mujoco.viewer
import numpy as np
ROOT=Path(__file__).resolve().parent
DT=.02; RADIUS=.05; TRACK=.21
CHARGE_DEMO_SECONDS=1.
wrap=lambda x:(x+np.pi)%(2*np.pi)-np.pi

def motion(x,ds,da):
    y=x.copy(); a=x[2]+da/2
    y[:2]+=ds*np.array([np.cos(a),np.sin(a)]); y[2]=wrap(x[2]+da)
    return y

class EKF:
    def __init__(self,r_scale=1.):
        self.r_scale=r_scale
        self.x=np.zeros(3); self.P=np.diag([.03,.03,.08])**2
        self.updates=0; self.rejected=0
    def predict(self,dl,dr):
        ds=(dl+dr)/2; da=(dr-dl)/TRACK; a=self.x[2]+da/2
        F=np.eye(3); F[0,2]=-ds*np.sin(a); F[1,2]=ds*np.cos(a)
        self.x=motion(self.x,ds,da)
        self.P=F@self.P@F.T+np.diag([2e-6,2e-6,8e-6])
    def update(self,z):
        V=self.r_scale*np.diag([.007,.007,.025])**2
        e=z-self.x; e[2]=wrap(e[2]); S=self.P+V
        if e@np.linalg.solve(S,e)>25: self.rejected+=1; return
        K=np.linalg.solve(S,self.P.T).T
        self.x+=K@e; self.x[2]=wrap(self.x[2])
        I=np.eye(3)-K; self.P=I@self.P@I.T+K@V@K.T
        self.updates+=1

class SensorSimulator:
    """Physical sensor simulation. Navigation never reads the simulated body pose."""
    DOWN_MOUNT=np.array([.14,0.,.60])
    FRONT_MOUNT=np.array([-.04,0.,.22])
    DOWN_RBC=np.diag([1.,-1.,-1.])
    FRONT_RBC=np.array([[0.,0.,1.],[-1.,0.,0.],[0.,-1.,0.]])
    def __init__(self,m,d,seed,dropout):
        self.m=m; self.d=d; self.rng=np.random.default_rng(seed); self.dropout=dropout
        self.robot=m.body('robot').id
        self.qadr=[int(m.joint(n+'_wheel_joint').qposadr[0]) for n in ('left','right')]
        self.renderer=mujoco.Renderer(m,height=480,width=480)
        self.detector=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))
        self.map=json.loads((ROOT/'landmark_map.json').read_text())
        self.encoder_scale=1.; self.blackout=None
        self.last=np.zeros(2); self.frames=0; self.tags=[]; self.rgb=None; self.front_rgb=None
        self.down_detections=0; self.front_detections=0; self.reprojection=[]
        self.angles=np.linspace(-np.pi,np.pi,120,endpoint=False)
    def encoders(self):
        q=np.round(self.d.qpos[self.qadr]*4096/(2*np.pi))*2*np.pi/4096
        delta=(q-self.last)*RADIUS; self.last=q
        return delta*self.encoder_scale+self.rng.normal(0,.000015,2)
    def image(self,name):
        self.renderer.update_scene(self.d,camera=name)
        rgb=self.renderer.render()
        noisy=np.clip(rgb.astype(float)+self.rng.normal(0,1,rgb.shape),0,255).astype(np.uint8)
        corners,ids,_=self.detector.detectMarkers(noisy)
        return rgb,corners,([] if ids is None else ids.ravel().tolist())
    def pnp(self,corners,ids,fovy,mount,Rbc):
        obj=[]; pixels=[]
        for c,i in zip(corners,ids):
            if str(i) in self.map['corners']:
                obj.extend(self.map['corners'][str(i)]); pixels.extend(c[0])
        if len(obj)<4: return None,None,None,None
        obj=np.asarray(obj,dtype=np.float64); pixels=np.asarray(pixels,dtype=np.float64)
        f=240/np.tan(np.deg2rad(fovy/2))
        K=np.array([[f,0,239.5],[0,f,239.5],[0,0,1.]])
        if len(obj)>=8:
            ok,rv,tv,inliers=cv2.solvePnPRansac(obj,pixels,K,None,reprojectionError=2.5,iterationsCount=100,flags=cv2.SOLVEPNP_ITERATIVE)
            if not ok or inliers is None or len(inliers)<8: return None,None,None,None
            inds=inliers.ravel(); obj=obj[inds]; pixels=pixels[inds]
            ok,rv,tv=cv2.solvePnP(obj,pixels,K,None,rv,tv,True,flags=cv2.SOLVEPNP_ITERATIVE)
        else:
            ok,rv,tv=cv2.solvePnP(obj,pixels,K,None,flags=cv2.SOLVEPNP_IPPE)
        if not ok: return None,None,None,None
        projected,_=cv2.projectPoints(obj,rv,tv,K,None)
        residual=float(np.sqrt(np.mean((projected.reshape(-1,2)-pixels)**2)))
        if residual>2.: return None,None,None,None
        Rcw=cv2.Rodrigues(rv)[0]; Rwb=Rcw.T@Rbc.T
        camera_pos=-Rcw.T@tv.ravel(); base=camera_pos-Rwb@mount
        if not(.02<base[2]<.18): return None,None,None,None
        self.reprojection.append(residual)
        return np.array([base[0],base[1],np.arctan2(Rwb[1,0],Rwb[0,0])]),Rcw,tv,Rbc
    def camera(self):
        self.rgb,corners,ids=self.image('onboard_down'); self.tags=ids; self.frames+=1
        pose,_,_,_=self.pnp(corners,ids,90,self.DOWN_MOUNT,self.DOWN_RBC)
        if pose is not None: self.down_detections+=1
        self.front_rgb,front_corners,front_ids=self.image('onboard_front')
        self.tags=ids+['front:'+str(i) for i in front_ids]
        dock=None
        if 10 in front_ids:
            j=front_ids.index(10)
            front_pose,Rcw,tv,_=self.pnp([front_corners[j]],[10],65,self.FRONT_MOUNT,self.FRONT_RBC)
            if front_pose is not None:
                # Measured station centre in robot frame, obtained from PnP pixels.
                camera_station=Rcw@np.array(self.map['centres']['10'])+tv.ravel()
                dock=(self.FRONT_MOUNT+self.FRONT_RBC@camera_station)[:2]
                self.front_detections+=1
                if pose is None: pose=front_pose
        if self.rng.random()<self.dropout or (self.blackout is not None and self.blackout[0]<=self.d.time<self.blackout[1]): pose=None; dock=None
        return pose,dock
    def lidar(self):
        origin=self.d.xpos[self.robot].copy(); origin[2]=.19
        rotation=self.d.xmat[self.robot].reshape(3,3)
        mask=np.ones(6,dtype=np.uint8); hit=np.zeros(1,dtype=np.int32); ranges=[]
        for a in self.angles:
            direction=rotation@np.array([np.cos(a),np.sin(a),0.]); direction[2]=0.; direction/=np.linalg.norm(direction)
            dist=mujoco.mj_ray(self.m,self.d,origin,direction,mask,1,self.robot,hit)
            ranges.append(3. if dist<0 else min(3.,max(.02,dist+self.rng.normal(0,.003))))
        return np.array(ranges)
    def close(self): self.renderer.close()

class Planner:
    """Occupancy map built exclusively from LiDAR endpoints and EKF pose."""
    def __init__(self):
        self.cell=.07; self.lo=np.array([-.7,-.6]); self.shape=(35,33)
        self.occupied=set(); self.scores={}; self.path=[]; self.replans=0
    def node(self,p): return tuple(np.rint((p-self.lo)/self.cell).astype(int))
    def point(self,n): return self.lo+np.array(n)*self.cell
    def scan(self,x,angles,ranges):
        free=set(); hits=set()
        for a,r in zip(angles,ranges):
            direction=np.array([np.cos(x[2]+a),np.sin(x[2]+a)])
            for distance in np.arange(.12,max(.12,r-.10),self.cell*.8):
                free.add(self.node(x[:2]+distance*direction))
            if r<2.9: hits.add(self.node(x[:2]+r*direction))
        for n in free-hits: self.scores[n]=max(-5,self.scores.get(n,0)-1)
        for n in hits: self.scores[n]=min(8,max(0,self.scores.get(n,0))+2)
        self.occupied={n for n,score in self.scores.items() if score>=2}
    def plan(self,x,goal,inflation=.245):
        blocked=set()
        for n in self.occupied:
            for i in range(-4,5):
                for j in range(-4,5):
                    if np.hypot(i,j)*self.cell<inflation: blocked.add((n[0]+i,n[1]+j))
        start=self.node(x[:2]); end=self.node(goal); blocked.discard(start)
        # A replanning margin must not trap a robot already inside inflated free space.
        # Keep measured surface cells blocked; only relax the artificial inflation locally.
        blocked={n for n in blocked if n in self.occupied or np.linalg.norm(self.point(n)-x[:2])>.14}
        queue=[(0,0,start)]; costs={start:0}; parent={}
        while queue:
            _,cost,n=heapq.heappop(queue)
            if n==end:
                out=[n]
                while n in parent: n=parent[n]; out.append(n)
                self.path=[self.point(p) for p in out[::-1]]; self.replans+=1; return True
            if cost>costs[n]+1e-9: continue
            for i,j in [(1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]:
                v=(n[0]+i,n[1]+j)
                if not(0<=v[0]<self.shape[0] and 0<=v[1]<self.shape[1]) or v in blocked: continue
                if i and j and ((n[0]+i,n[1]) in blocked or (n[0],n[1]+j) in blocked): continue
                c=cost+np.hypot(i,j)+(.08 if self.point(v)[1]<-.05 else 0)
                if c<costs.get(v,float('inf')):
                    costs[v]=c; parent[v]=n; heapq.heappush(queue,(c+np.linalg.norm(np.array(v)-end),c,v))
        self.path=[]; return False
    def target(self,x,goal):
        if not self.path: return goal
        nearest=int(np.argmin([np.linalg.norm(p-x[:2]) for p in self.path]))
        for p in self.path[nearest:]:
            if np.linalg.norm(p-x[:2])>.18: return p
        return goal

class FeedbackController:
    """Hybrid discrete LQR path tracker, plus turn-in-place alignment.
    Body-frame reference error e=[along,cross,yaw]; delta-u=-K e.
    """
    def __init__(self):
        self.A=np.eye(3); self.A[1,2]=.12*DT
        self.B=np.array([[-DT,0],[0,0],[0,-DT]])
        self.Q=np.diag([4.,14.,4.]); self.R=np.diag([2.,1.5])
        P=self.Q.copy()
        for _ in range(5000):
            K=np.linalg.solve(self.R+self.B.T@P@self.B,self.B.T@P@self.A)
            new=self.A.T@P@self.A-self.A.T@P@self.B@K+self.Q
            if np.max(np.abs(new-P))<1e-9: P=new; break
            P=new
        self.P=P; self.K=np.linalg.solve(self.R+self.B.T@P@self.B,self.B.T@P@self.A)
        self.eigenvalues=np.linalg.eigvals(self.A-self.B@self.K); self.calls=0
    def command(self,x,target,slow=False,heading=None):
        delta=target-x[:2]; distance=np.linalg.norm(delta)
        bearing=np.arctan2(delta[1],delta[0]); point_error=wrap(bearing-x[2])
        if abs(point_error)>.75: return 0.,float(np.clip(2.2*point_error,-1.,1.))
        ref=bearing if heading is None else heading
        e=np.array([np.cos(ref)*delta[0]+np.sin(ref)*delta[1],-np.sin(ref)*delta[0]+np.cos(ref)*delta[1],wrap(ref-x[2])])
        u=np.array([.06 if slow else .12,0.])-self.K@e
        self.calls+=1
        return float(np.clip(u[0],0,min(.10 if slow else .16,.8*distance))*max(0.,np.cos(point_error))),float(np.clip(u[1],-1.,1.))

class Battery:
    """Equivalent-circuit SOC charging model; virtual docking connector.
    2.2 Ah, 10.8..12.6 V OCV, 0.25 ohm battery resistance, RC transient.
    Electrical time accelerated 60x for visible simulation charging.
    """
    def __init__(self):
        self.soc=.40; self.initial_soc=self.soc; self.vrc=0.; self.voltage=11.52
        self.current=0.; self.connected=False; self.charged_ah=0.; self.charge_seconds=0.
    def tick(self,dt,connected,wheels):
        self.connected=connected; ocv=10.8+1.8*self.soc
        self.current=max(0.,min(1.,(12.6-ocv-self.vrc)/(.25+.75))) if connected else -(1.+.25*np.sum(np.abs(wheels)))/max(ocv,1.)
        edt=dt*60
        self.vrc+=edt*(.15*self.current-self.vrc)/20.
        self.soc=float(np.clip(self.soc+self.current*edt/(3600*2.2),0,1))
        self.voltage=ocv+.25*self.current+self.vrc
        if connected: self.charged_ah+=self.current*edt/3600; self.charge_seconds+=dt

class Evaluator:
    def __init__(self): self.errors=[]; self.yaw_errors=[]; self.contacts=0; self.clearance=3.
    def record(self,m,d,x,ranges):
        truth=d.xpos[m.body('robot').id].copy()
        self.errors.append(float(np.linalg.norm(x[:2]-truth[:2])))
        self.clearance=min(self.clearance,float(ranges.min()))
        for c in d.contact:
            names=[mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,int(g)) or '' for g in (c.geom1,c.geom2)]
            if any(n in ('obstacle','north','south','east','west','charger_base') for n in names): self.contacts+=1
        return truth

class Demo:
    def __init__(self,seed=7,dropout=0.,encoder_scale=1.,blackout=None,r_scale=1.):
        self.m=mujoco.MjModel.from_xml_path(str(ROOT/'track_a_scene.xml')); self.d=mujoco.MjData(self.m)
        self.seed=seed; self.dropout=dropout; self.encoder_scale=encoder_scale; self.blackout=blackout; self.r_scale=r_scale
        self.dofs=[int(self.m.joint(n+'_wheel_joint').dofadr[0]) for n in ('left','right')]
        self.motors=[self.m.actuator(n+'_motor').id for n in ('left','right')]
        self.sensors=SensorSimulator(self.m,self.d,seed,dropout); self.sensors.encoder_scale=encoder_scale; self.sensors.blackout=blackout; self.controller=FeedbackController()
        self.reset()
    def reset(self):
        mujoco.mj_resetData(self.m,self.d); mujoco.mj_forward(self.m,self.d)
        self.sensors.last=np.zeros(2); self.sensors.down_detections=0; self.sensors.front_detections=0; self.sensors.reprojection=[]; self.controller.calls=0; self.sensors.rng=np.random.default_rng(self.seed)
        self.battery=Battery(); self.charging_since=None; self.ekf=EKF(self.r_scale); self.odom=np.zeros(3); self.planner=Planner(); self.eval=Evaluator()
        self.goals=np.array([[1.,0.],[0.,1.],[0.,0.]])
        self.index=0; self.stage='SETTLING'; self.dock=None; self.dock_frames=0
        self.steps=0; self.hold=0.; self.rows=[]; self.waypoint_events=[]; self.eval_extra=[]; self.tracking=[]; self.effort=0.; self.ranges=np.full(120,3.)
        self.targets=np.zeros(2); self.stops=0; self.fault=''; self.camera_age=0.; self.station_age=-float('inf'); self.avoid=False
    def tick(self):
        dl,dr=self.sensors.encoders(); self.ekf.predict(dl,dr)
        self.odom=motion(self.odom,(dl+dr)/2,(dr-dl)/TRACK)
        if self.steps%10==0:
            z,dock=self.sensors.camera()
            if z is not None: self.ekf.update(z); self.camera_age=self.d.time
            if dock is not None:
                a=self.ekf.x[2]; rotation=np.array([[np.cos(a),-np.sin(a)],[np.sin(a),np.cos(a)]])
                measured=self.ekf.x[:2]+rotation@dock
                self.dock=measured if self.dock is None else .8*self.dock+.2*measured
                self.dock_frames+=1; self.station_age=self.d.time
        x=self.ekf.x.copy()
        if self.steps%10==0:
            self.ranges=self.sensors.lidar(); self.planner.scan(x,self.sensors.angles,self.ranges)
        v=w=0.
        if not np.isfinite(x).all(): self.fault='Non-finite estimate'
        if self.d.time>=239.99 and self.stage!='CHARGING': self.fault='Navigation timeout'
        if self.fault: self.stage='SAFETY STOP'
        elif self.stage=='CHARGING':
            position_error=np.linalg.norm(self.ekf.x[:2]-(self.dock+np.array([.20,0.])))
            if position_error>.09 or abs(wrap(np.pi-self.ekf.x[2]))>.12 or self.d.time-self.station_age>2:
                self.stage='DOCK ALIGN'; self.charging_since=None
        elif self.d.time<.7: self.stage='SETTLING'
        elif self.d.time<self.hold: self.stage='WAYPOINT HOLD'
        else:
            # A blocked plan must not repeatedly reuse an old station observation.
            if self.index>=len(self.goals) and self.stage=='BLOCKED' and self.d.time-self.station_age>2.:
                self.dock=None
            goal=self.goals[self.index] if self.index<len(self.goals) else (None if self.dock is None else self.dock+np.array([.20,0.]))
            if goal is None: self.stage='SEARCH DOCK'; w=.8
            else:
                distance=np.linalg.norm(goal-x[:2])
                tolerance=.075 if self.index<len(self.goals) else .06
                if distance<tolerance or (self.stage=='DOCK ALIGN' and distance<.08):
                    if self.index<len(self.goals):
                        self.waypoint_events.append(dict(index=self.index+1,time=self.d.time,goal=goal.tolist()))
                        self.index+=1; self.hold=self.d.time+.6; self.stage='WAYPOINT HOLD'
                        print(f'Waypoint {self.index}/{len(self.goals)} at {self.d.time:.1f}s',flush=True)
                    else:
                        # Dock facing west, aligned with known station orientation.
                        e=wrap(np.pi-x[2]); self.stage='DOCK ALIGN'; w=float(np.clip(2*e,-.5,.5))
                        if abs(e)<.07 and self.d.time-self.camera_age<.4 and self.d.time-self.station_age<.6:
                            self.stage='CHARGING'; self.charging_since=self.d.time; w=0.; print(f'Visual dock CHARGING at {self.d.time:.1f}s',flush=True)
                else:
                    if self.steps%25==0 or not self.planner.path:
                        if not self.planner.plan(x,goal,.17 if self.index>=len(self.goals) else .245): self.stage='BLOCKED'; self.stops+=1
                    if self.planner.path:
                        target=self.planner.target(x,goal)
                        self.avoid=any(abs(p[1])>.20 for p in self.planner.path) and self.index==0
                        self.stage='LIDAR DETOUR' if self.avoid else ('VISUAL DOCK' if self.index>=len(self.goals) else 'NAVIGATE')
                        if self.index>=len(self.goals) and distance<.25: target=goal
                        heading=None
                        if self.index<len(self.goals) and len(self.planner.path)>2:
                            nearest=int(np.argmin([np.linalg.norm(p-x[:2]) for p in self.planner.path]))
                            later=min(nearest+3,len(self.planner.path)-1)
                            tangent=self.planner.path[later]-self.planner.path[nearest]
                            if np.linalg.norm(tangent)>.03: heading=np.arctan2(tangent[1],tangent[0])
                        v,w=self.controller.command(x,target,self.index>=len(self.goals),heading)
                        front=self.ranges[np.abs(self.sensors.angles)<.35].min()
                        if v>0 and front<.24: v=0.; self.stops+=1; self.stage='LIDAR BRAKE'
        self.targets=np.clip([(v-w*TRACK/2)/RADIUS,(v+w*TRACK/2)/RADIUS],-6,6)
        # Feedback uses quantised/noisy encoder increments, never simulator qvel.
        measured_wheel_velocity=np.array([dl,dr])/(RADIUS*DT)
        torque=np.clip(.15*(self.targets-measured_wheel_velocity),-.20,.20)
        # Aligned evaluation: estimate and truth both at t_k, before advancing physics.
        truth=self.eval.record(self.m,self.d,x,self.ranges)
        rot=self.d.xmat[self.m.body('robot').id].reshape(3,3)
        truth_yaw=float(np.arctan2(rot[1,0],rot[0,0]))
        error=np.array([*(x[:2]-truth[:2]),wrap(x[2]-truth_yaw)])
        self.eval.yaw_errors.append(float(error[2]))
        nees=float(error@np.linalg.solve(self.ekf.P,error))
        track_error=None
        if self.planner.path and self.stage in ('NAVIGATE','LIDAR DETOUR','VISUAL DOCK'):
            pts=self.planner.path
            distances=[]
            for a,b in zip(pts[:-1],pts[1:]):
                ab=b-a; t=np.clip(np.dot(truth[:2]-a,ab)/max(np.dot(ab,ab),1e-12),0,1)
                distances.append(np.linalg.norm(truth[:2]-(a+t*ab)))
            if distances: track_error=float(min(distances)); self.tracking.append(track_error)
        self.effort+=float(torque@torque)*DT
        self.eval_extra.append([self.d.time,truth_yaw,nees,track_error,*torque,v,w])
        if self.waypoint_events and 'truth_error_m' not in self.waypoint_events[-1]:
            self.waypoint_events[-1]['truth_error_m']=float(np.linalg.norm(truth[:2]-np.array(self.waypoint_events[-1]['goal'])))
        self.rows.append([self.d.time,self.index,self.stage,*x,*self.odom,*truth[:2],self.ekf.updates,float(self.ranges.min()),*self.targets,*np.sqrt(np.diag(self.ekf.P)),self.battery.soc,self.battery.voltage,self.battery.current])
        self.battery.tick(DT,self.stage=='CHARGING',self.targets)
        for _ in range(10):
            self.d.ctrl[self.motors]=torque
            mujoco.mj_step(self.m,self.d)
        self.steps+=1
        if self.steps%500==0: print(f'{self.d.time:.1f}s: {self.stage}; vision={self.ekf.updates}; front-station={self.dock_frames}',flush=True)
    def complete(self):
        return self.stage=='CHARGING' and self.charging_since is not None and self.d.time-self.charging_since>=CHARGE_DEMO_SECONDS-1e-9
    def hud(self):
        return ('TRACK A: ONBOARD SENSORS -> EKF -> LQR\nMission\nRGB ArUco\nEKF position / heading\nLiDAR\nObstacle map / replans\nDock\nBattery / electrical model\nControls',
        f'Physical wheel torque; ground truth evaluation only\n{self.stage}{" (demo complete)" if self.complete() else ""}; waypoints {self.index}/{len(self.goals)}; t={self.d.time:.1f}s\nIDs {self.sensors.tags}; EKF updates {self.ekf.updates}\n{self.ekf.x[0]:.2f}, {self.ekf.x[1]:.2f} m / {np.rad2deg(self.ekf.x[2]):.1f} deg\n120 rays; nearest {self.ranges.min():.2f} m\n{len(self.planner.occupied)} cells / {self.planner.replans}\n{self.dock_frames} station detections; {"simulated charge" if self.stage=="CHARGING" else "pending"}\nSOC {100*self.battery.soc:.1f}% / {self.battery.voltage:.2f} V / {self.battery.current:.2f} A (electrical time x60)\nSPACE pause | R restart | ESC close')
    def close(self,out):
        out=str(Path(out) if Path(out).is_absolute() else ROOT/out)
        Path(out).parent.mkdir(parents=True,exist_ok=True)
        self.d.ctrl[:]=0
        if self.sensors.rgb is not None: cv2.imwrite(str(ROOT/f'{out}_camera.png'),cv2.cvtColor(self.sensors.rgb,cv2.COLOR_RGB2BGR))
        if self.sensors.front_rgb is not None: cv2.imwrite(str(ROOT/f'{out}_front_camera.png'),cv2.cvtColor(self.sensors.front_rgb,cv2.COLOR_RGB2BGR))
        self.sensors.close()
        with (ROOT/f'{out}.csv').open('w',newline='') as f:
            w=csv.writer(f); w.writerow(['time','reached','stage','ekf_x','ekf_y','ekf_yaw','odom_x','odom_y','odom_yaw','truth_x_evaluation_only','truth_y_evaluation_only','vision_updates','lidar_min','wheel_left','wheel_right','sigma_x','sigma_y','sigma_yaw','battery_soc','battery_voltage','battery_current']); w.writerows(self.rows)
        report=dict(charge_demo_seconds=CHARGE_DEMO_SECONDS,stage=self.stage,waypoints=self.index,waypoint_count=len(self.goals),waypoint_goals=self.goals.tolist(),time=self.d.time,camera_updates=self.ekf.updates,camera_rejected=self.ekf.rejected,dock_detections=self.dock_frames,rmse_m=float(np.sqrt((np.mean(np.array(self.eval.errors)**2) if self.eval.errors else 0.))),max_error_m=max(self.eval.errors,default=0),obstacle_contacts=self.eval.contacts,min_lidar_m=self.eval.clearance,lidar_brakes=self.stops,seed=self.seed,dropout=self.dropout,estimated_pose=self.ekf.x.tolist(),truth_xy_evaluation_only=self.d.xpos[self.m.body('robot').id,:2].tolist())
        report.update(ekf_covariance=self.ekf.P.tolist(),dock_position_error_m=None if self.dock is None else float(np.linalg.norm(self.ekf.x[:2]-(self.dock+np.array([.20,0.])))),dock_heading_error_rad=float(abs(wrap(np.pi-self.ekf.x[2]))),onboard_down_poses=self.sensors.down_detections,onboard_front_station_detections=self.sensors.front_detections,reprojection_rmse_px=float(np.mean(self.sensors.reprojection)) if self.sensors.reprojection else None,lqr_calls=self.controller.calls,lqr_closed_loop_eigenvalues=self.controller.eigenvalues.tolist(),battery_connected=self.battery.connected,battery_soc=self.battery.soc,battery_voltage=self.battery.voltage,battery_current=self.battery.current,charged_ah=self.battery.charged_ah,charge_seconds=self.battery.charge_seconds,dock_reference=None if self.dock is None else self.dock.tolist())
        with Path(f'{out}_evaluation.csv').open('w',newline='') as f:
            w=csv.writer(f); w.writerow(['time','truth_yaw_evaluation_only','nees','path_error_m','torque_left','torque_right','command_v','command_w']); w.writerows(self.eval_extra)
        # Final docking evaluation uses the scene reference only here, never in control.
        station=self.m.geom('station_marker').pos[:2]+np.array([.002,0.])
        final_rot=self.d.xmat[self.m.body('robot').id].reshape(3,3)
        final_yaw=float(np.arctan2(final_rot[1,0],final_rot[0,0]))
        report.update(dock_truth_error_m=float(np.linalg.norm(np.array(report['truth_xy_evaluation_only'])-(station+np.array([.20,0.])))),
            dock_truth_heading_error_rad=float(abs(wrap(np.pi-final_yaw))),
            odometry_rmse_m=float(np.sqrt(np.mean([(r[6]-r[9])**2+(r[7]-r[10])**2 for r in self.rows]))) if self.rows else None,
            yaw_rmse_rad=float(np.sqrt(np.mean(np.array(self.eval.yaw_errors)**2))) if self.rows else None,
            path_tracking_rmse_m=float(np.sqrt(np.mean(np.array(self.tracking)**2))) if self.tracking else None,
            mean_nees=float(np.mean([r[2] for r in self.eval_extra])) if self.rows else None,
            effort_torque_squared_integral=self.effort,waypoint_events=self.waypoint_events,
            encoder_scale=self.encoder_scale,blackout=self.blackout,r_scale=self.r_scale,
            timestamp_utc=datetime.now(timezone.utc).isoformat(),python=platform.python_version(),mujoco=mujoco.__version__,opencv=cv2.__version__,numpy=np.__version__,
            source_sha256={n:hashlib.sha256((ROOT/n).read_bytes()).hexdigest() for n in ['run_sim.py','track_a_scene.xml','landmark_map.json']})
        (ROOT/f'{out}.json').write_text(json.dumps(report,indent=2)); print(json.dumps(report,indent=2)); return report

class VideoRecorder:
    """Presentation/evaluation view, isolated from sensing and control."""
    def __init__(self,demo,out):
        self.demo=demo; self.path=ROOT/f'{out}_demo.mp4'; self.path.parent.mkdir(parents=True,exist_ok=True); self.frames=0
        self.writer=cv2.VideoWriter(str(self.path),cv2.VideoWriter_fourcc(*'mp4v'),10,(960,620))
        if not self.writer.isOpened(): raise RuntimeError('Video encoder unavailable')
    def frame(self):
        d=self.demo
        if d.steps%5: return
        d.sensors.renderer.update_scene(d.d,camera='overhead')
        overhead=d.sensors.renderer.render()
        front=d.sensors.front_rgb if d.sensors.front_rgb is not None else np.zeros_like(overhead)
        canvas=np.zeros((620,960,3),dtype=np.uint8)
        canvas[:480]=cv2.cvtColor(np.hstack([overhead,front]),cv2.COLOR_RGB2BGR)
        sigma=np.sqrt(np.diag(d.ekf.P)); truth=d.rows[-1][9:11]
        lines=[f'Track A | ONBOARD CAMERA -> EKF -> LQR | {d.stage} | {d.d.time:.1f}s | waypoint {d.index}/{len(d.goals)}',
               f'EKF x/y: {d.ekf.x[0]:.3f}, {d.ekf.x[1]:.3f} m | sigma: {sigma[0]:.3f}, {sigma[1]:.3f} m',
               f'Truth x/y (EVALUATION ONLY): {truth[0]:.3f}, {truth[1]:.3f} | front ArUco station frames: {d.dock_frames}',
               f'LiDAR min: {d.ranges.min():.2f}m | contacts: {d.eval.contacts} | SOC {d.battery.soc*100:.1f}% | {d.battery.voltage:.2f}V / {d.battery.current:.2f}A (x60)']
        for i,line in enumerate(lines): cv2.putText(canvas,line,(12,507+i*30),cv2.FONT_HERSHEY_SIMPLEX,.50,(240,240,240),1,cv2.LINE_AA)
        self.writer.write(canvas); self.frames+=1
    def close(self): self.writer.release()

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--headless',action='store_true'); ap.add_argument('--seconds',type=float,default=240); ap.add_argument('--smoke',action='store_true'); ap.add_argument('--seed',type=int,default=7); ap.add_argument('--dropout',type=float,default=0); ap.add_argument('--output',default='track_a_log'); ap.add_argument('--video',action='store_true'); ap.add_argument('--camera-preview',action='store_true'); ap.add_argument('--encoder-scale',type=float,default=1.); ap.add_argument('--blackout',nargs=2,type=float); ap.add_argument('--r-scale',type=float,default=1.); a=ap.parse_args()
    if not 0<=a.dropout<=1 or a.seconds<=0 or a.r_scale<=0 or a.encoder_scale<=0: ap.error('Invalid noise, duration or scale')
    demo=Demo(a.seed,a.dropout,a.encoder_scale,a.blackout,a.r_scale); recorder=VideoRecorder(demo,a.output) if a.video else None; controls={'pause':False,'reset':False}
    def key(k):
        if k==32: controls['pause']=not controls['pause']
        if k in (82,114): controls['reset']=True
    try:
        if a.headless:
            while demo.d.time<a.seconds and demo.stage!='SAFETY STOP' and not demo.complete(): demo.tick(); recorder.frame() if recorder else None
        else:
            with mujoco.viewer.launch_passive(demo.m,demo.d,key_callback=key,show_left_ui=False,show_right_ui=False) as viewer:
                with viewer.lock(): viewer.cam.lookat[:]=[.5,.5,0]; viewer.cam.distance=4; viewer.cam.azimuth=90; viewer.cam.elevation=-65
                while viewer.is_running() and (not a.smoke or demo.d.time<a.seconds):
                    begin=time.perf_counter()
                    with viewer.lock():
                        if controls['reset']: demo.reset(); controls['reset']=False
                        if not controls['pause'] and not demo.complete(): demo.tick(); recorder.frame() if recorder else None
                        left,right=demo.hud()
                    viewer.set_texts((mujoco.mjtFontScale.mjFONTSCALE_100,mujoco.mjtGridPos.mjGRID_TOPLEFT,left,right)); viewer.sync()
                    if a.camera_preview and demo.sensors.rgb is not None:
                        cv2.imshow('Onboard RGB: downward landmarks | forward charging station',cv2.cvtColor(np.hstack([demo.sensors.rgb,demo.sensors.front_rgb]),cv2.COLOR_RGB2BGR)); cv2.waitKey(1)
                    time.sleep(max(0,DT-(time.perf_counter()-begin)))
    finally:
        if recorder: recorder.close()
        cv2.destroyAllWindows()
        report=demo.close(a.output)
    if a.headless and (report['stage']!='CHARGING' or report['obstacle_contacts']): raise SystemExit(1)
if __name__=='__main__': main()






