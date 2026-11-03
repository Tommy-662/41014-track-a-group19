"""Create exactly 300 s of silent evidence video from fresh trials and cards.
This is a review draft; it has no narration and is not uploaded automatically.
"""
import sys,json,textwrap
from pathlib import Path
import numpy as np,cv2
from PIL import Image,ImageDraw,ImageFont
ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT.parent/'TrackA_Group19_5min_visual_draft.mp4'
W,H,FPS=1280,720,10
FONT=Path('C:/Windows/Fonts/arial.ttf'); BOLD=Path('C:/Windows/Fonts/arialbd.ttf')
def font(size,bold=False): return ImageFont.truetype(str(BOLD if bold else FONT),size)
def base(title,label='GROUP 19  |  ENQIAN TANG  |  25646922'):
    im=Image.new('RGB',(W,H),'#102d3b'); d=ImageDraw.Draw(im); d.text((50,25),label,font=font(18),fill='#8bd1e6'); d.text((50,65),title,font=font(34,True),fill='white'); d.line((50,123,1230,123),fill='#2d5364',width=2); return im
def card(title,bullets):
    im=base(title); d=ImageDraw.Draw(im); y=157
    for bullet in bullets:
        for j,line in enumerate(textwrap.wrap(bullet,width=78)):
            d.text((62 if j==0 else 88,y),('> ' if j==0 else '')+line,font=font(25),fill='#e1edf2'); y+=37
        y+=22
    d.text((50,675),'Simulation evidence | Ground truth is evaluation only | Silent visual draft',font=font(16),fill='#9eb9c6')
    return cv2.cvtColor(np.array(im),cv2.COLOR_RGB2BGR)
def chart(title,path):
    im=base(title); src=Image.open(path).convert('RGB'); src.thumbnail((1180,535)); im.paste(src,((W-src.width)//2,145)); return cv2.cvtColor(np.array(im),cv2.COLOR_RGB2BGR)
def main():
    writer=cv2.VideoWriter(str(OUT),cv2.VideoWriter_fourcc(*'mp4v'),FPS,(W,H)); assert writer.isOpened()
    n=0; sections=[]
    def still(frame,seconds,label):
        nonlocal n
        sections.append(dict(start=n/FPS,end=n/FPS+seconds,label=label))
        for _ in range(int(seconds*FPS)): writer.write(frame); n+=1
    def clip(path,seconds,label):
        nonlocal n
        sections.append(dict(start=n/FPS,end=n/FPS+seconds,label=label))
        c=cv2.VideoCapture(str(path)); assert c.isOpened(); last=None
        for i in range(int(seconds*FPS)):
            ok,f=c.read()
            if ok: last=f
            assert last is not None
            im=base(label,'THREE-WAYPOINT RUN: 3 NOV 2026 | NORMAL SPEED; FINAL FRAME HELD IF CLIP ENDS')
            # Keep every source HUD line, including truth and uncertainty.
            h,w=last.shape[:2]; scale=min(1180/w,570/h); frame=cv2.resize(last,(int(w*scale),int(h*scale)),interpolation=cv2.INTER_AREA)
            bg=cv2.cvtColor(np.array(im),cv2.COLOR_RGB2BGR); y=145; x=(W-frame.shape[1])//2; bg[y:y+frame.shape[0],x:x+frame.shape[1]]=frame
            writer.write(bg); n+=1
        c.release()
    still(card('Track A: waypoint navigation + visual docking',[
        '41014 Sensors and Control in Mechatronics Systems',
        'Three waypoints: (1,0), (0,1), then (0,0).',
        'Group 19: Enqian Tang (25646922), approved solo team.',
        'Onboard camera + wheel encoders + LiDAR -> EKF -> feedback.',
        'Simplified TurtleBot3-style model in MuJoCo. No hardware claim.'
    ]),20,'scope')
    still(card('State-space model and frame chain',[
        'State [world x, world y, yaw]. Inputs: forward speed and yaw rate.',
        'Differential drive: ds=(dL+dR)/2; d_yaw=(dR-dL)/0.21.',
        'OpenCV pixels -> PnP world-to-camera -> base pose -> EKF.',
        'Camera pose H=I: local observation rank 3. Encoders are relative.',
        'Moving-reference controllability rank 3; stationary rank 2.',
        'Truth -> evaluation logs/video only. Feedback uses measurements.'
    ]),30,'model_and_frames')
    still(card('Sensing, EKF and local LQR',[
        'Analytic simulation K/extrinsics; 28 held-out pose checks.',
        '50 Hz encoder prediction; 5 Hz camera and 120-ray LiDAR.',
        'EKF Q=diag(2e-6,2e-6,8e-6); R=diag(.007,.007,.025)^2.',
        'A-star obstacle path -> local LQR + turn mode -> encoder speed loop.',
        'LQR costs Q=diag(4,14,4), R=diag(2,1.5). Torque limited 0.20 N m.'
    ]),25,'sensing_and_control')
    c=cv2.VideoCapture(str(ROOT/'results/nominal_s7_demo.mp4')); nominal_seconds=int(np.ceil(c.get(cv2.CAP_PROP_FRAME_COUNT)/c.get(cv2.CAP_PROP_FPS)))+1; c.release()
    c=cv2.VideoCapture(str(ROOT/'results/dropout40_s7_demo.mp4')); disturbed_seconds=int(np.ceil(c.get(cv2.CAP_PROP_FRAME_COUNT)/c.get(cv2.CAP_PROP_FPS)))+1; c.release()
    clip(ROOT/'results/nominal_s7_demo.mp4',nominal_seconds,'Nominal trial | seed 7 | full mission')
    clip(ROOT/'results/dropout40_s7_demo.mp4',disturbed_seconds,'40% camera-observation dropout | seed 7')
    still(chart('12 repeated trials | three seeds per condition',ROOT/'results/comparison.png'),35,'repeated_results')
    still(chart('Blackout: growing uncertainty and recovery',ROOT/'results/uncertainty_video.png'),145-nominal_seconds-disturbed_seconds,'uncertainty')
    still(card('Retained failure and limits',[
        '100% observation loss: SEARCH DOCK at 50 s; failed mission.',
        'Estimated waypoint counter is not proof of true waypoint arrival.',
        'A one-second positive-current demonstration shows virtual charging.',
        'Distant oblique single-tag depth can be biased (max ~78.6 mm).',
        'One static scene, small samples; virtual charging and electrical time x60.'
    ]),25,'failure_and_limits')
    still(card('Reproducibility and individual responsibility',[
        'Group 19 | Enqian Tang | 25646922.',
        'Exact scene, locked dependencies, tests, raw logs and source hashes.',
        'Nominal mission checked with Python 3.11 and a clean Python 3.12 environment.',
        'One integrated sensing, estimation and control pipeline.',
        'Project page links the code, design and quantitative evidence.'
    ]),20,'contribution_and_reproduction')
    writer.release(); assert n==3000
    (ROOT/'results/video_timeline.json').write_text(json.dumps(dict(frames=n,fps=FPS,duration_s=n/FPS,audio=False,sections=sections),indent=2)); print(str(OUT))
if __name__=='__main__': main()
