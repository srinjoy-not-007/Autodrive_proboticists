import numpy as np
from scipy import ndimage; from PIL import Image
a=np.genfromtxt('data/racelines/finals_mesh_hyb675_race.csv',delimiter=',',names=True)
s=a['s_m']; xy=np.c_[a['x_m'],a['y_m']]; v=a['vx_mps'].copy()
n=len(s); tg=np.roll(xy,-1,0)-np.roll(xy,1,0); nrm=np.c_[-tg[:,1],tg[:,0]]/np.linalg.norm(tg,axis=1)[:,None]
# lateral shift toward the inside (left, +normal) of the s 37.5-40.5 corner, 1 m ramps
def bump(lo,hi,ramp): return np.clip(np.minimum((s-lo)/ramp,(hi-s)/ramp),0,1)
shift=0.08*bump(36.0,42.0,1.5)
from scipy.ndimage import gaussian_filter1d
sv=gaussian_filter1d(shift[:,None]*nrm,2.0,axis=0,mode='wrap')
xy2=xy+sv
# re-derive s, psi, kappa
d=np.linalg.norm(np.roll(xy2,-1,0)-xy2,axis=1); s2=np.r_[0,np.cumsum(d[:-1])]
tg2=np.roll(xy2,-1,0)-np.roll(xy2,1,0); psi2=np.arctan2(tg2[:,1],tg2[:,0])
dpsi=np.diff(np.unwrap(np.r_[psi2,psi2[0]])); kappa2=dpsi/((d+np.roll(d,1))/2)
# outside the shifted zone keep the file's own psi/kappa (my central difference doubles the hairpin)
zz=shift>0; kappa2=np.where(zz,kappa2,a['kappa_radpm']); psi2=np.where(zz,psi2,a['psi_rad'])
# cap exit speed s 38.3-39.8 at 3.0 (was 2.47->3.55), then forward-pass smooth with ax<=6.5 and brake<=4.0
cap=np.where(bump(38.3,39.8,0.3)>0,3.0,99.0); v=np.minimum(v,cap)
zone=(s>35.5)&(s<42.5); v=np.where(zone,np.minimum(v,np.sqrt(6.0/np.maximum(np.abs(kappa2),1e-3))),v)
for _ in range(3):
    for i in range(n):  # forward accel limit
        j=(i+1)%n; v[j]=min(v[j],np.sqrt(v[i]**2+2*6.5*d[i]))
    for i in range(n-1,-1,-1):  # backward brake limit
        j=(i+1)%n; v[i]=min(v[i],np.sqrt(v[j]**2+2*4.0*d[i]))
vv=np.r_[v,v[0]]; ax=(vv[1:]**2-vv[:-1]**2)/(2*d)
hdr=open('data/racelines/finals_mesh_race.csv').readline().strip()[2:]
np.savetxt('data/racelines/finals_mesh_hyb675b_race.csv',np.c_[s2,xy2,psi2,kappa2,v,ax],delimiter=',',fmt='%.4f',header=hdr,comments='# ')
img=np.array(Image.open('data/maps/finals_mesh.pgm')); res=0.025; ox,oy=-4.0,-17.575; dist=ndimage.distance_transform_edt(img>=128)*res
w=lambda P:np.array([dist[img.shape[0]-1-int((p[1]-oy)/res),int((p[0]-ox)/res)] for p in P])
print('ideal lap %.2f (hyb675 %.2f); length %.2f -> %.2f'%((d/v).sum(),(d/a['vx_mps']).sum(),s[-1]+d[-1],s2[-1]+d[-1]))
print('line-to-wall min s36-41: before %.3f after %.3f ; global min before %.3f after %.3f'%(w(xy[(s>36)&(s<41)]).min(),w(xy2[(s>36)&(s<41)]).min(),w(xy).min(),w(xy2).min()))
print('kappa max s36-41 before %.2f after %.2f'%(abs(a['kappa_radpm'][(s>36)&(s<41)]).max(),abs(kappa2[(s>36)&(s<41)]).max()))
for ss in (37.5,38,38.5,39,39.5): i=np.argmin(abs(s-ss)); print(' s=%.1f v %.2f->%.2f k %+.2f->%+.2f'%(ss,a['vx_mps'][i],v[i],a['kappa_radpm'][i],kappa2[i]))
