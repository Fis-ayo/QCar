"""
curvature_check.py  (PC only)

Camera report §64: lane curvature seen by the front camera. On saved front frames well
inside an arc (the whole 0-0.6 m view in the arc), the lane points are refitted with a
constant test curvature 0.60 ... 2.40 /m instead of the map's; the best total rms per
region and drive is printed, with the map arcs for comparison. Control: the 2 -> 4 arc.
Drive list in the script.
    python qcar/curvature_check.py
"""
import sys, os, numpy as np, cv2
HERE=os.path.dirname(os.path.abspath(__file__)); sys.path.insert(0,HERE); sys.path.insert(0,os.path.join(HERE,'..','..','libraries','python'))
import lane_camera as lc
from c3_replay import load_vc
from hal.products.mats import SDCSRoadMap
vc=load_vc(); route=vc['RoutePath'](vc['remove_backward_points'](SDCSRoadMap(leftHandTraffic=False).generate_path(vc['nodeSequence'])))
RL=route.L
ss=np.arange(0,RL,0.01); k=route.curvature(ss)
def segs():
    out=[];i=0
    while i<len(ss):
        j=i
        while j+1<len(ss) and abs(k[j+1]-k[i])<0.05: j+=1
        if ss[j]-ss[i]>0.4 and abs(k[i])>0.3: out.append((round(ss[i],2),round(ss[j],2),round(float(np.median(k[i:j+1])),3)))
        i=j+1
    return out
print('map arcs (start, end, curvature):', segs())
KS=np.round(np.arange(0.6,2.41,0.05),2)
A='logs/car A'
drives=[('C',A+'/20261120_driveC','20261120_194525'),('D',A+'/20261120_driveD','20261120_212931'),('E',A+'/20261120_driveE','20261120_215026'),('F',A+'/20261120_driveF','20261120_223328')]
regions=[('2->4 arc (control)',None),('roundabout first half',(7.66,8.45)),('roundabout second half',(8.45,9.2))]
arcs=segs()
a24=[a for a in arcs if 2.0<a[0]<3.5][0]
regions[0]=('2->4 arc (control), map k %.2f'%a24[2],(a24[0]+0.05,a24[1]-0.65))
for name,F,tag in drives:
    o=np.genfromtxt(F+'/odometry_drive.csv',delimiter=',',names=True,dtype=None,encoding=None)
    fr=sorted(float(f[:-10]) for f in os.listdir(F+'/'+tag) if f.endswith('_front.jpg'))
    S=np.interp(fr,o['t_epoch'],o['cs_s'])
    line=[]
    for lab,(lo,hi) in regions:
        tot=np.zeros(len(KS)); n=0; per=[]
        sign=np.sign(route.curvature(np.array([(lo+hi)/2]))[0])
        for t,s in zip(fr,S):
            if not (lo<=s%RL<hi) or s-o['cs_s'][0]<0.5 or s-o['cs_s'][0]>12: continue
            out,dbg=lc.process(cv2.imread(os.path.join(F,tag,f'{t:.3f}_front.jpg'))); y,e=lc.boundary_points(dbg)
            r=[]
            for kk in KS:
                c,nn=lc.lane_from_profile(np.full(len(lc.PROFILE_S),sign*kk)); acc=m=0
                for p,side in ((y,lc.LANE_W/2),(e,-lc.LANE_W/2)):
                    if len(p)<lc.MIN_POINTS//2: continue
                    if len(p)>lc.MF_MAX_PTS: p=p[np.linspace(0,len(p)-1,lc.MF_MAX_PTS).astype(int)]
                    dd,hh,rr=lc.fit_boundary(p,c,nn,side)
                    if abs(dd)<lc.MF_MAX_D and abs(hh)<lc.MF_MAX_H: acc+=rr**2*len(p); m+=len(p)
                r.append(np.sqrt(acc/m) if m else np.nan)
            r=np.array(r)
            if np.all(np.isnan(r)) or np.nanmin(r)>0.3: continue
            tot+=np.nan_to_num(r**2,nan=1.0); n+=1; per.append(KS[np.nanargmin(r)])
        line.append(f"{lab}: best k {KS[np.argmin(tot)]:.2f} (frames {n}, per-frame median {np.median(per) if per else np.nan:.2f})" if n else f"{lab}: no frames")
    print(name,' | '.join(line), flush=True)
