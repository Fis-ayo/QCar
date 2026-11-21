"""
scale_check.py  (PC only)

Camera report §60: encoder-to-map distance ratio k (map metres per RAW encoder metre,
i.e. without enc_scale) from the recorded drives, two ways:
  k_cam: from the last accepted GPS readings before the 4 -> 14 GPS failure (s 3.5-4.6)
         to the painted roundabout entry, located by the front camera (map lane shape
         refitted with s shifted, frames s 7.0-7.46 before any along-route correction);
  k_gps: GPS along-route position vs raw encoder distance from the start to s 4.2.
Drive list is in the script.
    python qcar/scale_check.py
"""
import sys, os, re, numpy as np, cv2
HERE=os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0,HERE); sys.path.insert(0,os.path.join(HERE,'..','..','libraries','python'))
import lane_camera as lc
from c3_replay import load_vc
from hal.products.mats import SDCSRoadMap
vc=load_vc(); RL=None
route=vc['RoutePath'](vc['remove_backward_points'](SDCSRoadMap(leftHandTraffic=False).generate_path(vc['nodeSequence'])))
RL=route.L; GA=vc['gps_front_ahead']
A='logs/car A'
drives=[('§46',A+'/Fresh_Log','20261119_234304'),('A',A+'/20261120_driveA','20261120_190909'),('B',A+'/20261120_driveB','20261120_193052'),
        ('C',A+'/20261120_driveC','20261120_194525'),('ruler',A+'/20261120_rulercheck','20261120_201634'),
        ('D',A+'/20261120_driveD','20261120_212931'),('E',A+'/20261120_driveE','20261120_215026')]
def entry_shift(folder, o, t_max):
    fr=sorted(float(f[:-10]) for f in os.listdir(folder) if f.endswith('_front.jpg'))
    S=np.interp(fr,o['t_epoch'],o['cs_s'])
    SH=np.round(np.arange(-0.30,0.301,0.02),2); tot=np.zeros(len(SH)); n=0; tf=[]
    for t,s in zip(fr,S):
        if not (7.0<=s%RL<7.46) or s-o['cs_s'][0]<5 or t>t_max: continue
        out,dbg=lc.process(cv2.imread(os.path.join(folder,f'{t:.3f}_front.jpg'))); y,e=lc.boundary_points(dbg)
        r=[]
        for sh in SH:
            c,nn=lc.lane_from_profile(route.curvature(s+sh+lc.PROFILE_S)); acc=m=0
            for p,side in ((y,lc.LANE_W/2),(e,-lc.LANE_W/2)):
                if len(p)<lc.MIN_POINTS//2: continue
                if len(p)>lc.MF_MAX_PTS: p=p[np.linspace(0,len(p)-1,lc.MF_MAX_PTS).astype(int)]
                dd,hh,rr=lc.fit_boundary(p,c,nn,side); acc+=rr**2*len(p); m+=len(p)
            r.append(np.sqrt(acc/m) if m else np.nan)
        r=np.array(r)
        if np.all(np.isnan(r)): continue
        tot+=np.nan_to_num(r**2); n+=1; tf.append(t)
    return SH[np.argmin(tot)], n, (np.mean(tf) if tf else np.nan)
print(f'{"drive":6s} {"GPS anchor s":>12s} {"entry s_true":>12s} {"enc raw":>8s} {"k_cam":>6s} | {"k_gps (start->4.2)":>18s} n')
ks_cam=[]; ks_gps=[]
for name,folder,tag in drives:
    o=np.genfromtxt(folder+'/odometry_drive.csv',delimiter=',',names=True,dtype=None,encoding=None)
    t=o['t_epoch']; enc=o['dist_m']   # raw encoder distance
    g=[]
    for line in open(folder+'/vehicle_control.log'):
        m_=re.match(r'GPS: \(([-\d.]+),([-\d.]+),([-\d]+)deg\).*t_epoch=([\d.]+)', line)
        if m_: g.append((float(m_.group(4)),float(m_.group(1)),float(m_.group(2))))
    g=np.array(g); g=g[(g[:,0]>=t[0])&(g[:,0]<=t[-1])]
    # GPS front-axle station of each accepted reading, near the logged s
    sl=np.interp(g[:,0],t,o['cs_s'])
    sg=np.array([route.project(x,y,s-GA,0.4)[0]+GA for (_,x,y),s in zip(g,sl)])
    sg=sl+((sg-sl+RL/2)%RL-RL/2)          # unwrap near the logged s
    eg=np.interp(g[:,0],t,enc)
    first=(sl-o['cs_s'][0])<6.5            # first pass, before the roundabout
    # GPS-only regression from the start to s 4.2 (moving)
    mv=first&((sl%RL<4.2)|(sl%RL>13.0))&(eg>0.05)
    kg=np.polyfit(eg[mv],sg[mv],1)[0] if mv.sum()>5 else np.nan
    # anchor: median of the last 5 accepted readings before s 4.6
    an=np.where(first&(sl%RL>3.5)&(sl%RL<4.6))[0]
    an=an[-5:]
    # correction time (D/E): only frames before it
    tcorr=np.inf
    for line in open(folder+'/vehicle_control.log'):
        m_=re.search(r'along-route correction.*t_epoch=([\d.]+)',line)
        if m_: tcorr=min(tcorr,float(m_.group(1)))
    sh,nfr,tfr=entry_shift(folder+'/'+tag,o,tcorr)
    if len(an)==0 or nfr==0:
        print(f'{name:6s} no anchor/frames'); continue
    s_a=np.median(sg[an]-eg[an])           # station minus raw encoder at the anchor (offset of a line with slope 1)
    # true entry station and raw encoder there
    s_e=np.interp(tfr,t,o['cs_s'])+sh; e_e=np.interp(tfr,t,enc)
    # k from anchor (mean of the anchor readings) to entry
    e_a=np.mean(eg[an]); s_an=np.mean(sg[an])
    k=(s_e-s_an)/(e_e-e_a)
    ks_cam.append(k); ks_gps.append(kg)
    print(f'{name:6s} {s_an%RL:12.2f} {s_e%RL:12.2f} {e_e-e_a:8.2f} {k:6.3f} | {kg:18.3f} {mv.sum()}   (entry shift {sh:+.2f} m, {nfr} frames)')
print(f'k_cam mean {np.mean(ks_cam):.3f} sd {np.std(ks_cam):.3f}; k_gps mean {np.nanmean(ks_gps):.3f} sd {np.nanstd(ks_gps):.3f}; car uses {vc["enc_scale"]}')
