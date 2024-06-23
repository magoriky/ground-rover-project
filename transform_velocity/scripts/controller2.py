import numpy as np
from numpy import cos
from numpy import sin
from numpy import abs
from numpy import sign
def controller(x,y,th,v,w,xd,yd,xr,yr,xdr,ydr,xddr,yddr):
    lnd1 = 5
    lnd2 = 5
    K1=200
    K2 =200
    e = 0.3
    s1 = lnd1*(x-xr)+ (xd-xdr)
    s2 = lnd2*(y-yr)+(yd-ydr)
    d =0.17 
    c1 = -(1/7.7094)
    c2 = -(1/7.6847)
    c3 = 12.0991*c1
    c4 = 61.5953*c1
    c5 = -206.0774*c2
    c6 = 175.5830*c2
    

    THWR = lnd1*(v*cos(th) -d*w*sin(th) -xdr) + ((c3/c2)*(w**2) + (c4/c1)*v)*cos(th) -v*w*sin(th) +d*((c5/c2)*v*w + (c6/c2)*w)*sin(th) - d*(w**2)*cos(th) - xddr
    THVR = lnd2*(v*sin(th) +d*w*cos(th)-ydr)+((c3/c1)*(w**2) + (c4/c1)*v)*sin(th)+v*w*cos(th)-d((c5/c2)*v*w + (c6/c2)*w)*cos(th) -d*(w**2)*sin(th)-yddr

    if abs(s1) <= e:
        S11 = s1/e
    else:
        S11 = sign(s1)

    if abs(s2) <= e:
        S22 = s2/e
    else:
        S22=sign(s2)
        
    u_hat1 = -THWR - K1*S11
    u_hat2 = -THVR - K2*S22
    A = [-d*sin(th)/c2, cos(th)/c1;
             d*cos(th)/c2, sin(th)/c1]
    U = np.linalg.inv(A).dot(np.array([[u_hat1],[u_hat2]]))
    wr = U[0,0]
    vr = U[1,1]    

    return [vr,wr]