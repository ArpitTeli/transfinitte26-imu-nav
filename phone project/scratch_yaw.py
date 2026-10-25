import math, numpy as np, sys
from engine import Engine
G=9.80665; HZ=50; rng=np.random.default_rng(1)
B=np.array([20.,0.,-42.])
def run(params, T=40):
    e=Engine(mapping="Flat on palm, top edge = forward", params=params); yaw=0
    for k in range(int(T*HZ)):
        t=k/HZ; w=t>6
        rate=0.5*math.sin(2*math.pi*(t-6)/20) if w else 0.
        yaw+=rate/HZ
        acc=(rng.normal(0,.05),(0.8*math.sin(2*math.pi*1.8*t) if w else 0)+rng.normal(0,.05),G+rng.normal(0,.05))
        gyr=(rng.normal(0,.003),rng.normal(0,.003),rate+0.004+rng.normal(0,.003))
        c,s=math.cos(-yaw),math.sin(-yaw)
        mag=(c*B[0]-s*B[1],s*B[0]+c*B[1],B[2])
        e.feed("gyr",t,*gyr); e.feed("mag",t,*mag); e.feed("acc",t,*acc)
    return math.degrees(yaw), math.degrees(e.last["yaw"]), e.last["mag_ok"], e.last["mag_rej"]
for name,p in [("default",{}),("nomag",{"mag":0}),("loose",{"mag_tol":0.5,"dip_tol":0.2,"step_tol":0.5}),("tuned",{"r_mag":5,"mag_ref_s":2,"step_tol":0.3,"dip_tol":0.03,"mag_tol":0.15})]:
    print(name, [round(x,1) for x in run(p)])
