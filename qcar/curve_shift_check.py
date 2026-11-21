"""
curve_shift_check.py  (PC only)

Camera report §55: along-route position error at a curve entry, measured by the front
camera. For each saved front frame before a curve, the map lane shape is refitted with
the car's logged s shifted by -0.30 ... +0.30 m (lane_camera.boundary_points /
fit_boundary, all rms kept); the shift with the lowest rms over all frames is where the
painted lane actually starts curving relative to the car's estimate (+ = the lane curves
earlier than expected = the estimate is behind, or the map's curve starts later than the
paint). Control: the 0 -> 2 entry, where the GPS still holds s (expect ~0).

    python qcar/curve_shift_check.py "<name>|<raw frames folder>|<odometry_drive.csv>" [...]
"""
import sys, os, numpy as np, cv2
HERE=os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0,HERE); sys.path.insert(0,os.path.join(HERE,'..','..','libraries','python'))
import lane_camera as lc
from c3_replay import load_vc
from hal.products.mats import SDCSRoadMap
vc=load_vc()
route=vc['RoutePath'](vc['remove_backward_points'](SDCSRoadMap(leftHandTraffic=False).generate_path(vc['nodeSequence'])))
RL=route.L
SH=np.round(np.arange(-0.30,0.301,0.02),2)
def scan(folder, odo, lo, hi, first_pass_m=12):
    o=np.genfromtxt(odo,delimiter=',',names=True,dtype=None,encoding=None)
    fr=sorted(float(f[:-10]) for f in os.listdir(folder) if f.endswith('_front.jpg'))
    S=np.interp(fr,o['t_epoch'],o['cs_s'])
    tot=np.zeros(len(SH)); n=0; best=[]
    for t,s in zip(fr,S):
        sm=s%RL
        if not (lo<=sm<hi) or s-o['cs_s'][0]>first_pass_m: continue
        out,dbg=lc.process(cv2.imread(os.path.join(folder,f'{t:.3f}_front.jpg')))
        y,e=lc.boundary_points(dbg)
        r=[]
        for sh in SH:
            c,nn=lc.lane_from_profile(route.curvature(s+sh+lc.PROFILE_S))
            acc=0; m=0
            for p,side in ((y,lc.LANE_W/2),(e,-lc.LANE_W/2)):
                if len(p)<lc.MIN_POINTS//2: continue
                if len(p)>lc.MF_MAX_PTS: p=p[np.linspace(0,len(p)-1,lc.MF_MAX_PTS).astype(int)]
                dd,hh,rr=lc.fit_boundary(p,c,nn,side)
                acc+=rr**2*len(p); m+=len(p)
            r.append(np.sqrt(acc/m) if m else np.nan)
        r=np.array(r)
        if np.all(np.isnan(r)): continue
        tot+=np.nan_to_num(r**2); n+=1
        best.append((sm,SH[np.nanargmin(r)],np.nanmin(r)*100,np.nanmax(r)*100))
    return tot,n,best
for name,folder,odo in [(a,b,c) for a,b,c in [x.split('|') for x in sys.argv[1:]]]:
    for lab,lo,hi in (('0->2 entry (GPS good)',13.20,13.74+0.0),('roundabout entry',7.0,7.62)):
        if lab.startswith('0->2'):
            tot,n,best=scan(folder,odo,13.2,13.74)
            t2,n2,b2=scan(folder,odo,0.0,0.30); tot=tot+t2; n+=n2; best+=b2
        else:
            tot,n,best=scan(folder,odo,lo,hi)
        if n==0: print(f'{name:5s} {lab}: no frames'); continue
        mv=[x for k,x in enumerate(best) if k==0 or abs(x[0]-best[k-1][0])>0.005]   # frames while moving
        pf=' '.join(f'{sm:.2f}:{b:+.2f}' for sm,b,_,_ in mv)
        print(f'{name:5s} {lab:22s}: frames {n:2d}, best shift (all frames) {SH[np.argmin(tot)]:+.2f} m | moving frames s:shift {pf}')
