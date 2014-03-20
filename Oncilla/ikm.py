# -*- coding: utf-8 -*-
"""
Created on Wed Mar 27 14:59:27 2013

@author: michael
"""
from __future__ import division
from scipy.optimize import fsolve
from math import sin,cos,pi,sqrt,atan,atan2,tan
import time
Width=140 #afstand tss L- en R-poot
Length=227 #afstand tss voor- en achterpoot

A=60. #dijlengte (mm) front
B=56. #scheenlengte front
C=62.5 #voetlengte front
amin_f=-43./180.*pi #commando 1000
amax_f=62./180.*pi #commando 0
bmin_f=85./180.*pi #commando 1000
bmax_f=135./180.*pi ##commando 0

D=78. #dijlengte (mm) rear
E=65. #scheenlengte rear
F=52. #voetlengte rear
amin_h=-43./180.*pi#commando 1000
amax_h=58./180.*pi#commando 0
bmin_h=78./180.*pi#commando 1000
bmax_h=135./180.*pi#commando 0

#hoeken tov de x-as waartussen (x,y) moet liggen
anglemin_f=-56./180.*pi
anglemax_f=36./180.*pi
anglemin_h=-56./180.*pi
anglemax_h=28./180.*pi

# hoeken die de poten maken tov de verticaal door gebruik servo's
gmin=[(90+8.98)/180.*pi,(90-8.98)/180.*pi,(90+6.38)/180.*pi,(90-6.38)/180.*pi]  #servocommando 0: poten naar buiten
gmax=[(90-8.08)/180.*pi,(90+8.08)/180.*pi,(90-10.40)/180.*pi,(90+10.40)/180.*pi] #servocommando 1: poten naar binnen



def calc_ground_angles(angle): 
    #Calculates the ground angles for the four legs as function of 1 turning angle in radians,
    #as to make the robot turn around 1 point.
    if angle==0:
        ground_angles=[0]*4
    else:
        alpha=abs(angle)
        beta=pi/2-atan(tan(pi/2-alpha)+2*Width/Length)
        if angle>0:
            ground_angles=[beta,alpha,-beta,-alpha]
        else:
            ground_angles=[-alpha,-beta,alpha,beta]
    return ground_angles
    
def calc_a(a0,angle): 
    #Calculates the step length for each leg, so the legs on the inner side of the turn go slower,
    #as to make the robot turn around 1 point.
    if angle==0:
        a=[a0]*4
    else:
        alpha=abs(angle)
        x=Length/2*atan(pi/2-alpha)
        r1=sqrt(x**2+(Length/2)**2)
        r2=sqrt((x+Width)**2+(Length/2)**2)
        if angle>0:
            a=[a0,a0/r2*r1]*2
        else:
            a=[a0/r2*r1,a0]*2
    return a
def scale(x,xmin,xmax,ymin,ymax):
    return (x-xmin+0.0)/(xmax-xmin+0.0)*(ymax-ymin+0.0)+ymin

'''def angle2command_f((alpha,beta)):
    #converts angles in radians to robot commands (0-1000) for the front legs
    #Deprecated with new robot interface
    return scale(alpha,amin_f,amax_f,1000,0),scale(beta,bmin_f,bmax_f,1000,0)

def angle2command_h((alpha,beta)):
    #converts angles in radians to robot commands (0-1000) for the hind legs
    #Deprecated with new robot interface
    return scale(alpha,amin_h,amax_h,1000,0),scale(beta,bmin_h,bmax_h,1000,0)

def command2angle_f((hip,knee)):
    #converts robot commands (0-1000) to angles in radians for the front legs
    #Deprecated with new robot interface
    return scale(hip,1000,0,amin_f,amax_f),scale(knee,1000,0,bmin_f,bmax_f)

def command2angle_h((hip,knee)):
    #converts robot commands (0-1000) to angles in radians for the hind legs
    #Deprecated with new robot interface
    return scale(hip,1000,0,amin_h,amax_h),scale(knee,1000,0,bmin_h,bmax_h)
        
def full_angle2command(leg,(alpha,beta,gamma)):
    #converts angles in radians to robot commands (0-1000 for motors, 0-1 for servos)
    #Deprecated with new robot interface
    if leg<2:
        return scale(alpha,amin_f,amax_f,1000,0),scale(beta,bmin_f,bmax_f,1000,0),scale(gamma,gmin[leg],gmax[leg],0,1)
    else:
        return scale(alpha,amin_h,amax_h,1000,0),scale(beta,bmin_h,bmax_h,1000,0),scale(gamma,gmin[leg],gmax[leg],0,1)

def full_command2angle(leg,(hip,knee,servo)):
    #converts robot commands (0-1000 for motors, 0-1 for servos) to angles in radians
    #Deprecated with new robot interface
    if leg<2:
        return scale(hip,1000,0,amin_f,amax_f),scale(knee,1000,0,bmin_f,bmax_f),scale(servo,0,1,gmin[leg],gmax[leg])
    else:
        return scale(hip,1000,0,amin_h,amax_h),scale(knee,1000,0,bmin_h,bmax_h),scale(servo,0,1,gmin[leg],gmax[leg])'''     

def fkm_f(alpha,beta):
    #Forward kinematics front legs
    al=max(min(alpha,amax_f),amin_f)
    be=max(min(beta,bmax_f),bmin_f)
    return (A+C)*cos(al)-B*cos(al+be),(A+C)*sin(al)-B*sin(al+be)

def fkm_h(alpha,beta):
    #Forward kinematics front legs
    al=max(min(alpha,amax_h),amin_h)
    be=max(min(beta,bmax_h),bmin_h)
    return (D+F)*cos(al)-E*cos(al+be),(D+F)*sin(al)-E*sin(al+be)
    
def full_fkm(leg,alpha,beta,gamma=pi/2):
    #Forward kinematics (leg from 0-3)
    
    if leg==0 or leg==2:#restore servo angle to convention (90 degrees=vertical and positive angle=leg to the right)
        gamma=-gamma        
    gamma=gamma+pi/2
    
    if leg<2:
        x,y=fkm_f(alpha,beta)
    else:
        x,y=fkm_h(alpha,beta)
    gammamin=min(gmax[leg],gmin[leg])
    gammamax=max(gmax[leg],gmin[leg])
    ga=max(min(gamma,gammamax),gammamin)
    return x*sin(ga),y,x*cos(ga)
        
def fkm_f_nonlimiting(al,be):
    return (A+C)*cos(al)-B*cos(al+be),(A+C)*sin(al)-B*sin(al+be)
    
def fkm_h_nonlimiting(al,be):
    return (D+F)*cos(al)-E*cos(al+be),(D+F)*sin(al)-E*sin(al+be)
    
def ikm_f(x,y):    
    #front leg inverse kinematics, x and y in mm, x-axis from hip pointing down, y-axis pointing forward, angles in radians
    x+=0.0     #cast to double
    y+=0.0        
    xymax=fkm_f(0,bmax_f)
    xymin=fkm_f(0,bmin_f)
    rmax=sqrt(xymax[0]**2+xymax[1]**2)
    rmin=sqrt(xymin[0]**2+xymin[1]**2)
    r=sqrt(x**2+y**2)
    
    if x<=0:
        print "pootpositie met xwaarde <=0 gevraagd!"
        return 0,bmax_f
    elif r>rmax:
        x=x/r*rmax
        y=y/r*rmax
    elif r<rmin:
        x=x/r*rmin
        y=y/r*rmin
        
    angle=atan2(y,x)
    if angle>anglemax_f or angle<anglemin_f:    
        if angle<anglemin_f:
            theta=anglemin_f-angle
        else: 
            theta=anglemax_f-angle #negative angle
            
        x2=x*cos(theta)-y*sin(theta)
        y2=x*sin(theta)+y*cos(theta)
        x=x2
        y=y2
        
    def equations(p):
        res=fkm_f_nonlimiting(p[0],p[1])
        return (res[0]-x,res[1]-y)

    alpha, beta =  fsolve(equations, (0, (bmax_f+bmin_f)/2), maxfev=10000)
    alpha=((alpha+pi)%(2*pi))-pi #get them in -pi..pi range
    beta=((beta+pi/2)%(2*pi))-pi/2 # get them in -pi/2..3*pi/2 range
    if beta>pi: #solutions that bend the knee in the wrong direction are handled here
        beta = 2*pi-beta
        alpha = 2*atan2(y,x)-alpha
    alpha=max(min(alpha,amax_f),amin_f)
    beta=max(min(beta,bmax_f),bmin_f)
    return alpha,beta
    
def ikm_h(x,y):    #hind leg inverse kinematics, x and y in mm, x-axis from hip pointing down, y-axis pointing forward, angles in radians
    x+=0.0    
    y+=0.0
    xymax=fkm_h(0,bmax_h)
    xymin=fkm_h(0,bmin_h)
    rmax=sqrt(xymax[0]**2+xymax[1]**2)
    rmin=sqrt(xymin[0]**2+xymin[1]**2)
    r=sqrt(x**2+y**2)
    
    if x<=0:
        print "pootpositie met xwaarde <=0 gevraagd!"
        return 0,bmax_h
    elif r>rmax:
        x=x/r*rmax
        y=y/r*rmax
    elif r<rmin:
        x=x/r*rmin
        y=y/r*rmin
        
    angle=atan2(y,x)
    if angle>anglemax_h or angle<anglemin_h:    
        if angle<anglemin_h:
            theta=anglemin_h-angle
        else: 
            theta=anglemax_h-angle #negative angle
            
        x2=x*cos(theta)-y*sin(theta)
        y2=x*sin(theta)+y*cos(theta)
        x=x2
        y=y2
    
    def equations(p):
        res=fkm_h_nonlimiting(p[0],p[1])
        return (res[0]-x,res[1]-y)

    alpha, beta =  fsolve(equations, (0, (bmax_h+bmin_h)/2), maxfev=10000)
    alpha=(alpha+pi)%(2*pi)-pi #get them in -pi..pi range
    beta=(beta+pi/2)%(2*pi)-pi/2 # get them in -pi/2..3*pi/2 range
    if beta>pi: #solutions that bend the knee in the wrong direction are handled here
        beta = 2*pi-beta
        alpha = 2*atan2(y,x)-alpha
    alpha=max(min(alpha,amax_h),amin_h)
    beta=max(min(beta,bmax_h),bmin_h)
    return alpha,beta
    
def full_ikm(leg,x,y,z): #leg from 0 to 3
    gamma=atan2(x,z)
    if gamma>pi or gamma<0:
        print "poot boven heup gevraagd!"
    gammamin=min(gmax[leg],gmin[leg])
    gammamax=max(gmax[leg],gmin[leg])
    
    if gamma>gammamax or gamma<gammamin:    
        if gamma<gammamin:
            z=x/tan(gammamin)
        else: 
            z=x/tan(gammamax)

    gamma=atan2(x,z)        

    if leg<2:
        alpha,beta=ikm_f(sqrt(x**2+z**2),y)
    else:
        alpha,beta=ikm_h(sqrt(x**2+z**2),y)
    
    gamma=gamma-pi/2 #change angles to robot convention: 0 degrees=vertical and positive angle = leg outside
    if leg==0 or leg==2:
        gamma=-gamma
        
    return alpha,beta,gamma

"""Onderstaande contouren staan nu in gait_contours.py"""
    
def half_ellipse(theta,x0,y0,a,b,stance,angle=0): #a: half length of ellipse, b: height of ellipse, x0: distance of stance below hip joint, y0: offset of stance line in relation to hip joint (0=centered, positive= to the front), stance: percentage of period that foot is on ground
    stancerad=stance*2*pi
    theta2=(theta+stancerad/2)%(2*pi)
    
    if theta2<stancerad:
        x=x0
        y=-theta2/stancerad*2*a+a+y0

    else:
        x=-b*sin((theta2-stancerad)/(2*pi-stancerad)*pi)+x0
        y=-a*cos((theta2-stancerad)/(2*pi-stancerad)*pi)+y0
    z=0
    z0=0
    y=y-y0
    z=z-z0
    y2=y*cos(angle)-z*sin(angle)
    z2=y*sin(angle)+z*cos(angle)
    return x,y2+y0,z2+z0

def spline(t,px,py):
    x=t**3*(-px[0]+3*px[1]-3*px[2]+px[3])+t**2*(3*px[0]-6*px[1]+3*px[2])+t*(-3*px[0]+3*px[1])+(1*px[0])
    y=t**3*(-py[0]+3*py[1]-3*py[2]+py[3])+t**2*(3*py[0]-6*py[1]+3*py[2])+t*(-3*py[0]+3*py[1])+(1*py[0])
    return x,y
    
def double_spline(theta,x0,y0,a,l,stance,angle=0):
    stancerad=stance*2*pi

    theta2=(theta+stancerad/2)%(2*pi)
    if theta2<stancerad:
        t=theta2/stancerad
        px=[x0,x0+l[2],x0+l[3],x0]
        py=[a+y0,a+y0,-a+y0,-a+y0]
        
    else:
        t=(theta2-stancerad)/(2*pi-stancerad)
        px=[x0,x0-l[0],x0-l[1],x0]
        py=[-a+y0,-a+y0,a+y0,a+y0]
    
    x,y=spline(t,px,py)
    z=0
    z0=0
    y=y-y0
    z=z-z0
    y2=y*cos(angle)-z*sin(angle)
    z2=y*sin(angle)+z*cos(angle)
    return x,y2+y0,z2+z0

def line(theta, x0, y0, h, stance, angle=0):#angle = ??
    stancerad=stance*2*pi
    x=0
    y=0
    z=0
    theta2=(theta+stancerad/2)%(2*pi) # schuif hoek op om niet met overgang 2pi -> 0 te zitten in midden van standfase, maar tussen de 2 fasen
    if theta2<stancerad:
        #stand
        x=x0+h/2
        y=y0
        
    else:
        y=y0
        factor=(theta2-stancerad)/(2.*pi-stancerad)
        if factor < 1/2:
            x=x0+h/2-h*2.*factor # go up
        else:
            x=x0+h/2-h+2.*h*(factor-1/2) # go down
    
    return x,y,z
    
if __name__ == '__main__':
    '''filename="ellipse_f.csv"
    text_file = open(filename, "w")
    text_file.write("theta;x;y\n")
    for x in xrange(0,201):
        theta=x*1.0/100.0*pi
        a,b=ikm_f(*half_ellipse(theta,138.45,0.39,76,13.69,0.5))
        x,y=fkm_f(a,b)
        text_file.write("%f;%f;%f;\n"%(theta,x,y))

    text_file.close()
    
    filename="ellipse_h.csv"
    text_file = open(filename, "w")
    text_file.write("theta;x;y\n")
    for x in xrange(0,201):
        theta=x*1.0/100.0*pi
        a,b=ikm_h(*half_ellipse(theta,138.45,-6.83,76,26.12,0.5))
        x,y=fkm_h(a,b)
        text_file.write("%f;%f;%f;\n"%(theta,x,y))

    text_file.close()
    '''

    '''filename="km.csv"
    text_file = open(filename, "w")
        
    text_file.write("alpha;beta;x;y\n")
    for alpha in xrange(-20,20):
        for beta in xrange(0,40):
            a=alpha/20.*pi/2.
            b=beta/40.*pi
            text_file.write("%f;%f;%f;%f\n"%(a,b,fkm_h(a,b)[0],fkm_h(a,b)[1]))
            #print a,b,fkm_f(a,b)

    text_file.close()
    print "done1"
    
    filename="ikm_hind.csv"
    text_file = open(filename, "w")
    text_file.write("x;y;alpha;beta;x';y'\n")
    for x in xrange(1,200,2):
        for y in xrange(-200,200,2):
            #if(128<=sqrt(x*x+y*y)<=170):
            xy=ikm_h(x,y)
            text_file.write("%f;%f;%f;%f;%f;%f\n"%(x,y,xy[0],xy[1],fkm_h(xy[0],xy[1])[0],fkm_h(xy[0],xy[1])[1]))
    text_file.close()
    print "done2"
    '''
    t0=time.time()
    filename="ikm_front.csv"
    text_file = open(filename, "w")
    text_file.write("x;y;alpha;beta;x';y'\n")
    for x in xrange(1,200,2):
        for y in xrange(-200,200,2):
            xy=ikm_f(x,y)
            text_file.write("%f;%f;%f;%f;%f;%f\n"%(x,y,xy[0],xy[1],fkm_f(xy[0],xy[1])[0],fkm_f(xy[0],xy[1])[1]))
    text_file.close()
    print "Verstreken tijd: %f s"%(time.time()-t0)
    print "Tijd per berekening: %f ms"%((time.time()-t0)/200/100*1000)
    print "done3"
    
    filename="splines.csv"
    text_file = open(filename, "w")
    text_file.write("omega;x;y;z\n")
    for n in xrange(0,1000,1):
            theta=n/500.0*pi
            t=n/1000.
            x,y,z =double_spline(theta,0,0,1,[1,2,3,4],0.4,0)
            #x,y =spline(t,[0,-1,-3,0],[-1,-1,1,1])            
            #print t,x,y
            text_file.write("%f;%f;%f;%f\n"%(theta,x,y,z))
    print "done"      
    text_file.close()

