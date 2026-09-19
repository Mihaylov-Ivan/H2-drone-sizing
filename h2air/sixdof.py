"""6-DOF/control screening 6-DOF rigid-body model for H2AIR-4+1.

Body axes: x forward, y right, z down. Earth axes: NED.
The module is intentionally explicit about fidelity limits: derivatives and
actuator data not supported by the current source package remain engineering
assumptions and are carried into result warnings/gates.
"""
from __future__ import annotations
from dataclasses import dataclass, asdict
from math import asin, atan2, cos, degrees, pi, radians, sin, sqrt
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple
import numpy as np
from .airfoils import interpolate_polar
from scipy.optimize import least_squares, lsq_linear

G = 9.80665
EPS = 1e-9


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def quat_normalize(q: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(q)
    return q / max(n, EPS)


def quat_from_euler(phi: float, theta: float, psi: float) -> np.ndarray:
    cr, sr = cos(phi/2), sin(phi/2)
    cp, sp = cos(theta/2), sin(theta/2)
    cy, sy = cos(psi/2), sin(psi/2)
    return quat_normalize(np.array([
        cr*cp*cy + sr*sp*sy,
        sr*cp*cy - cr*sp*sy,
        cr*sp*cy + sr*cp*sy,
        cr*cp*sy - sr*sp*cy,
    ], dtype=float))


def euler_from_quat(q: np.ndarray) -> Tuple[float,float,float]:
    w,x,y,z = quat_normalize(q)
    sinr = 2*(w*x+y*z); cosr=1-2*(x*x+y*y)
    phi=atan2(sinr,cosr)
    sint=clamp(2*(w*y-z*x),-1,1); theta=asin(sint)
    siny=2*(w*z+x*y); cosy=1-2*(y*y+z*z); psi=atan2(siny,cosy)
    return phi,theta,psi


def rot_body_to_ned(q: np.ndarray) -> np.ndarray:
    w,x,y,z = quat_normalize(q)
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]
    ], dtype=float)


def quat_dot(q: np.ndarray, omega: np.ndarray) -> np.ndarray:
    p,rq,r = omega
    w,x,y,z = q
    return 0.5*np.array([
        -x*p-y*rq-z*r,
        w*p+y*r-z*rq,
        w*rq-x*r+z*p,
        w*r+x*rq-y*p,
    ])


def wrap_pi(a: float) -> float:
    return (a+pi)%(2*pi)-pi


def phase3_station_to_body(station: Sequence[float], cg_x: float, cg_z_up: float) -> np.ndarray:
    sx, sy, sz_up = map(float, station)
    return np.array([cg_x-sx, sy, cg_z_up-sz_up], dtype=float)


@dataclass
class State:
    t: float
    pos_ned: np.ndarray
    vel_body: np.ndarray
    quat_bn: np.ndarray
    omega_body: np.ndarray
    lift_thrusts: np.ndarray
    pusher_rpm: float
    surfaces: np.ndarray  # da,de,dr [rad]
    speed_integral: float = 0.0

    def copy(self) -> "State":
        return State(self.t,self.pos_ned.copy(),self.vel_body.copy(),self.quat_bn.copy(),self.omega_body.copy(),self.lift_thrusts.copy(),float(self.pusher_rpm),self.surfaces.copy(),float(self.speed_integral))


@dataclass
class AeroResult:
    Va: float; alpha: float; beta: float; qbar: float
    CL: float; CD: float; CY: float; Cl: float; Cm: float; Cn: float
    force_body: np.ndarray; moment_body: np.ndarray


@dataclass
class Controls:
    lift_thrust_cmd: np.ndarray
    pusher_rpm_cmd: float
    surface_cmd: np.ndarray
    desired_moment_body: np.ndarray
    rotor_moment_cmd: np.ndarray
    surface_moment_cmd: np.ndarray
    rotor_allocation_residual: float


@dataclass
class Sample:
    time_s: float; north_m: float; east_m: float; altitude_m: float
    u_mps: float; v_mps: float; w_mps: float; airspeed_mps: float
    roll_deg: float; pitch_deg: float; yaw_deg: float
    p_deg_s: float; q_deg_s: float; r_deg_s: float
    alpha_deg: float; beta_deg: float
    lift_total_N: float; pusher_rpm: float
    lift_T1_N: float; lift_T2_N: float; lift_T3_N: float; lift_T4_N: float
    aileron_deg: float; elevator_deg: float; rudder_deg: float
    max_rotor_utilization: float; max_surface_utilization: float
    target_pitch_deg: float; target_speed_mps: float; target_altitude_m: float
    pitch_tracking_error_deg: float; altitude_error_m: float
    rotor_allocation_residual: float
    CL: float
    clmax_utilization: float

    def to_dict(self) -> Dict[str,Any]: return asdict(self)


class Model:
    def __init__(self,cfg: Mapping[str,Any], static_margin_override: Optional[float]=None):
        self.cfg=cfg; a=cfg['aircraft']; ad=cfg['aerodynamics']
        self.m=float(a['mass_kg']); self.S=float(a['wing_area_m2']); self.b=float(a['wing_span_m']); self.c=float(a['wing_mac_m'])
        self.I=np.array(a['inertia_tensor_kgm2'],dtype=float); self.Iinv=np.linalg.inv(self.I)
        self.rho=float(ad['rho_kg_m3']); self.CLalpha=float(ad['CL_alpha_per_rad']); self.alpha0=radians(float(ad['zero_lift_alpha_deg']))
        self.AR=self.b*self.b/self.S; self.k=1/(pi*self.AR*float(ad['oswald_efficiency']))
        cg=float(a['cg_station_x_m']); np_x=float(a['neutral_point_station_x_m'])
        self.static_margin=(np_x-cg)/self.c if static_margin_override is None else float(static_margin_override)
        self.Cma=-self.CLalpha*self.static_margin
        self.motor_r=[phase3_station_to_body(s,cg,float(a['cg_z_up_m'])) for s in a['lift_motor_stations_m']]
        self.rotation=np.array(a['lift_motor_rotation_sign'],dtype=float)
        self.r_pusher=phase3_station_to_body(a['pusher_station_m'],cg,float(a['cg_z_up_m']))
        self.tmax=float(cfg['lift_system']['max_thrust_per_motor_N']); self.kq=float(cfg['lift_system']['reaction_torque_per_thrust_m'])
        self._Arot=self._rotor_matrix()

    def _rotor_matrix(self)->np.ndarray:
        A=np.zeros((4,4)); A[0,:]=1.0
        for i,r in enumerate(self.motor_r):
            # r x [0,0,-T] = [-y*T, x*T, 0]
            A[1,i]=-r[1]; A[2,i]=r[0]; A[3,i]=self.rotation[i]*self.kq
        return A

    def aero(self,state: State, wind_ned: np.ndarray)->AeroResult:
        R=rot_body_to_ned(state.quat_bn); wind_b=R.T@wind_ned; vrel=state.vel_body-wind_b
        u,v,w=vrel; Va=float(np.linalg.norm(vrel)); Va=max(Va,0.05)
        alpha=atan2(w,max(abs(u),1e-6)); beta=asin(clamp(v/Va,-1,1)); qbar=.5*self.rho*Va*Va
        ad=self.cfg['aerodynamics']; da,de,dr=state.surfaces; p,q,r=state.omega_body
        ph=p*self.b/(2*Va); qh=q*self.c/(2*Va); rh=r*self.b/(2*Va)
        polar_point = interpolate_polar(ad['polar_rows'], alpha*180.0/pi) if ad.get('polar_rows') else None
        if polar_point is not None:
            CL=float(polar_point['CL'])
            CD=float(polar_point['CD'])+self.k*CL*CL+0.05*beta*beta
        else:
            CL=clamp(self.CLalpha*(alpha-self.alpha0),float(ad['CL_min']),float(ad['CL_max']))
            CD=float(ad['CD0_folded'])+self.k*CL*CL+0.05*beta*beta
        CY=float(ad['CY_beta'])*beta+float(ad['CY_p'])*ph+float(ad['CY_r'])*rh+float(ad['CY_dr'])*dr
        Cl=float(ad['Cl_beta'])*beta+float(ad['Cl_p'])*ph+float(ad['Cl_r'])*rh+float(ad['Cl_da'])*da+float(ad['Cl_dr'])*dr
        airfoil_cm=float(polar_point.get('CM',0.0)) if polar_point is not None else float(ad['Cm0'])
        Cm=airfoil_cm+self.Cma*(alpha-self.alpha0)+float(ad['Cm_q'])*qh+float(ad['Cm_de'])*de
        Cn=float(ad['Cn_beta'])*beta+float(ad['Cn_p'])*ph+float(ad['Cn_r'])*rh+float(ad['Cn_da'])*da+float(ad['Cn_dr'])*dr
        L=qbar*self.S*CL; D=qbar*self.S*CD; Y=qbar*self.S*CY
        ca,sa=cos(alpha),sin(alpha)
        F=np.array([-D*ca+L*sa, Y, -D*sa-L*ca],dtype=float)
        M=np.array([qbar*self.S*self.b*Cl,qbar*self.S*self.c*Cm,qbar*self.S*self.b*Cn],dtype=float)
        return AeroResult(Va,alpha,beta,qbar,CL,CD,CY,Cl,Cm,Cn,F,M)

    def pusher_point(self,speed: float,rpm: float)->Tuple[float,float]:
        # Same generic map basis as Mission/transition, implemented locally to keep 6DOF self-contained.
        D=float(self.cfg['pusher']['propeller_diameter_in'])*0.0254; n=max(0,rpm)/60.0
        if n<1e-6:return 0.0,0.0
        J=max(0,speed)/(n*D)
        # Aero/propulsion generic propeller map (still an engineering assumption).
        table=[(0.0,0.12,0.07),(0.2,0.115,0.064),(0.4,0.105,0.060),(0.6,0.080,0.0615),(0.8,0.055,0.0564),(1.0,0.025,0.0455),(1.15,0.0,0.035)]
        if J<=table[0][0]: Ct,Cp=table[0][1],table[0][2]
        elif J>=table[-1][0]: Ct,Cp=table[-1][1],table[-1][2]
        else:
            for (j0,ct0,cp0),(j1,ct1,cp1) in zip(table,table[1:]):
                if j0<=J<=j1:
                    f=(J-j0)/(j1-j0); Ct=ct0+(ct1-ct0)*f; Cp=cp0+(cp1-cp0)*f; break
        T=Ct*self.rho*n*n*D**4; Pshaft=Cp*self.rho*n**3*D**5
        eta_m=float(self.cfg['pusher'].get('motor_efficiency_assumption', 0.82))
        eta_e=float(self.cfg['pusher'].get('esc_efficiency_assumption', 0.95))
        Pelec=Pshaft/max(eta_m*eta_e, EPS)
        pmax=float(self.cfg['pusher']['max_electrical_power_W'])
        if Pelec>pmax:
            ratio=pmax/Pelec; T*=ratio; Pelec=pmax
        return T,Pelec

    def forces_moments(self,state:State,wind_ned:np.ndarray)->Tuple[np.ndarray,np.ndarray,AeroResult,float,float]:
        ar=self.aero(state,wind_ned); R=rot_body_to_ned(state.quat_bn)
        Fg=R.T@np.array([0,0,self.m*G],dtype=float)
        Tlift=float(np.sum(state.lift_thrusts)); Fl=np.array([0,0,-Tlift],dtype=float)
        Ml=np.zeros(3)
        for Ti,ri,si in zip(state.lift_thrusts,self.motor_r,self.rotation):
            Ml += np.cross(ri,np.array([0,0,-Ti])) + np.array([0,0,si*self.kq*Ti])
        Tp,Pp=self.pusher_point(ar.Va,state.pusher_rpm); Fp=np.array([Tp,0,0],dtype=float); Mp=np.cross(self.r_pusher,Fp)
        return ar.force_body+Fg+Fl+Fp, ar.moment_body+Ml+Mp, ar, Tp, Pp

    def derivative(self,state:State,wind_ned:np.ndarray)->Tuple[np.ndarray,np.ndarray,np.ndarray,np.ndarray]:
        F,M,_,_,_=self.forces_moments(state,wind_ned); omega=state.omega_body
        vdot=F/self.m-np.cross(omega,state.vel_body)
        odot=self.Iinv@(M-np.cross(omega,self.I@omega))
        qdot=quat_dot(state.quat_bn,omega); pdot=rot_body_to_ned(state.quat_bn)@state.vel_body
        return pdot,vdot,qdot,odot

    def actuator_update(self,state:State,ctrl:Controls,dt:float)->None:
        ls=self.cfg['lift_system']; tau=float(ls['motor_time_constant_s']); rate=float(ls['max_thrust_rate_N_s'])
        raw=(ctrl.lift_thrust_cmd-state.lift_thrusts)/max(tau,dt); raw=np.clip(raw,-rate,rate); state.lift_thrusts=np.clip(state.lift_thrusts+raw*dt,0,self.tmax)
        p=self.cfg['pusher']; taup=float(p['rpm_time_constant_s']); state.pusher_rpm += (ctrl.pusher_rpm_cmd-state.pusher_rpm)/max(taup,dt)*dt; state.pusher_rpm=clamp(state.pusher_rpm,0,float(p['max_rpm']))
        cs=self.cfg['control_surfaces']; limits=np.radians([float(cs['aileron_limit_deg']),float(cs['elevator_limit_deg']),float(cs['rudder_limit_deg'])]); rates=np.radians([float(cs['aileron_rate_deg_s']),float(cs['elevator_rate_deg_s']),float(cs['rudder_rate_deg_s'])]); taus=float(cs['servo_time_constant_s'])
        srate=(ctrl.surface_cmd-state.surfaces)/max(taus,dt); srate=np.clip(srate,-rates,rates); state.surfaces=np.clip(state.surfaces+srate*dt,-limits,limits)

    def integrate(self,state:State,dt:float,wind_ned:np.ndarray)->None:
        # RK4 on rigid-body states, actuators frozen within substep.
        y0=np.concatenate([state.pos_ned,state.vel_body,state.quat_bn,state.omega_body])
        def f(y):
            st=state.copy(); st.pos_ned=y[0:3]; st.vel_body=y[3:6]; st.quat_bn=quat_normalize(y[6:10]); st.omega_body=y[10:13]
            pd,vd,qd,od=self.derivative(st,wind_ned); return np.concatenate([pd,vd,qd,od])
        k1=f(y0); k2=f(y0+.5*dt*k1); k3=f(y0+.5*dt*k2); k4=f(y0+dt*k3); y=y0+dt*(k1+2*k2+2*k3+k4)/6
        state.pos_ned=y[0:3]; state.vel_body=y[3:6]; state.quat_bn=quat_normalize(y[6:10]); state.omega_body=y[10:13]; state.t+=dt

    def rotor_allocate(self,total_thrust:float,moment_cmd:np.ndarray)->Tuple[np.ndarray,float]:
        b=np.array([total_thrust,*moment_cmd],dtype=float)
        # Normalize rows to comparable dimensionless priorities; collective slightly higher weight.
        scales=np.array([max(self.m*G,1),max(self.m*G*.48,1),max(self.m*G*.25,1),max(self.m*G*self.kq,1)])
        W=np.diag(np.array([2.0,1.0,1.0,0.7])/scales)
        res=lsq_linear(W@self._Arot,W@b,bounds=(np.zeros(4),np.full(4,self.tmax)),lsmr_tol='auto')
        achieved=self._Arot@res.x; residual=float(np.linalg.norm(W@(achieved-b)))
        return res.x,residual

    def trim_cruise(self,speed_mps:float=20.0)->Dict[str,float]:
        # solve alpha, elevator, pusher thrust for X,Z,M_y equilibrium, wings level
        def fun(x):
            alpha,de,Tp=x; q=quat_from_euler(0,alpha,0)
            # Level earth-relative flight at angle of attack alpha: body velocity has w=V sin(alpha).
            st=State(0,np.array([0,0,-30.0]),np.array([speed_mps*cos(alpha),0,speed_mps*sin(alpha)]),q,np.zeros(3),np.zeros(4),0,np.array([0,de,0]))
            ar=self.aero(st,np.zeros(3)); R=rot_body_to_ned(q); Fg=R.T@np.array([0,0,self.m*G]); F=ar.force_body+Fg+np.array([Tp,0,0]); M=ar.moment_body+np.cross(self.r_pusher,np.array([Tp,0,0]))
            return [F[0]/self.m,F[2]/self.m,M[1]/max(self.I[1,1],.1)]
        sol=least_squares(fun,np.array([radians(6),radians(5),12.0]),bounds=([radians(-5),radians(-25),0],[radians(18),radians(25),80]))
        alpha,de,Tp=sol.x
        return {'success':bool(sol.cost<1e-6),'alpha_deg':degrees(alpha),'elevator_deg':degrees(de),'pusher_thrust_N':Tp,'residual_norm':float(np.linalg.norm(fun(sol.x))),'static_margin':self.static_margin,'Cm_alpha_per_rad':self.Cma}


class Controller:
    def __init__(self,model:Model): self.m=model
    def command(self,state:State,target:Mapping[str,float],wind_ned:np.ndarray,dt:float)->Controls:
        cfg=self.m.cfg; cc=cfg['controller']; phi,theta,psi=euler_from_quat(state.quat_bn); p,q,r=state.omega_body
        roll_ref=radians(clamp(float(target.get('roll_deg',0)),-float(cc['max_roll_command_deg']),float(cc['max_roll_command_deg'])))
        pitch_ref=radians(clamp(float(target.get('pitch_deg',0)),-float(cc['max_pitch_command_deg']),float(cc['max_pitch_command_deg'])))
        yaw_ref=radians(float(target.get('yaw_deg',0)))
        acc=np.array([
            float(cc['roll_kp_rad_s2_per_rad'])*(roll_ref-phi)-float(cc['roll_kd_rad_s2_per_rad_s'])*p,
            float(cc['pitch_kp_rad_s2_per_rad'])*(pitch_ref-theta)-float(cc['pitch_kd_rad_s2_per_rad_s'])*q,
            float(cc['yaw_kp_rad_s2_per_rad'])*wrap_pi(yaw_ref-psi)-float(cc['yaw_kd_rad_s2_per_rad_s'])*r,
        ])
        Mdes=self.m.I@acc+np.cross(state.omega_body,self.m.I@state.omega_body)
        ar=self.m.aero(state,wind_ned); Va=ar.Va
        blend=clamp((Va-float(cc['surface_blend_start_mps']))/max(float(cc['surface_blend_full_mps'])-float(cc['surface_blend_start_mps']),EPS),0,1)
        # Invert control derivatives to provide the requested blend of moment through aerodynamic surfaces.
        ad=cfg['aerodynamics']; qS=max(ar.qbar*self.m.S,EPS)
        surface=np.zeros(3); Msurf=np.zeros(3)
        # aileron, elevator, rudder independently; coupled lateral derivative is handled by dynamics.
        if blend>0:
            denom_roll=qS*self.m.b*float(ad['Cl_da']); denom_pitch=qS*self.m.c*float(ad['Cm_de']); denom_yaw=qS*self.m.b*float(ad['Cn_dr'])
            surface[0]=blend*Mdes[0]/denom_roll if abs(denom_roll)>EPS else 0
            surface[1]=blend*Mdes[1]/denom_pitch if abs(denom_pitch)>EPS else 0
            surface[2]=blend*Mdes[2]/denom_yaw if abs(denom_yaw)>EPS else 0
        cs=cfg['control_surfaces']; lim=np.radians([float(cs['aileron_limit_deg']),float(cs['elevator_limit_deg']),float(cs['rudder_limit_deg'])]); surface=np.clip(surface,-lim,lim)
        Msurf=np.array([qS*self.m.b*float(ad['Cl_da'])*surface[0],qS*self.m.c*float(ad['Cm_de'])*surface[1],qS*self.m.b*float(ad['Cn_dr'])*surface[2]])
        Mrot=Mdes-Msurf
        # altitude loop, NED altitude = -down
        alt=-state.pos_ned[2]; vz_ned=(rot_body_to_ned(state.quat_bn)@state.vel_body)[2]; vz_up=-vz_ned
        alt_ref=float(target.get('altitude_m',alt)); vz_ref=float(target.get('vertical_speed_mps',0))
        aup=clamp(float(cc['altitude_kp_mps2_per_m'])*(alt_ref-alt)+float(cc['vertical_speed_kd_mps2_per_mps'])*(vz_ref-vz_up),-float(cc['max_commanded_vertical_accel_mps2']),float(cc['max_commanded_vertical_accel_mps2']))
        # approximate wing vertical support from current aero force transformed to NED (negative down = upward)
        Fa_ned=rot_body_to_ned(state.quat_bn)@ar.force_body; aero_up=-Fa_ned[2]
        phi_now,theta_now,_=euler_from_quat(state.quat_bn); vertical_per_thrust=max(cos(phi_now)*cos(theta_now),0.25)
        totalT=clamp((self.m.m*(G+aup)-aero_up)/vertical_per_thrust,0,4*self.m.tmax)
        thrusts,resid=self.m.rotor_allocate(totalT,Mrot)
        # speed loop -> required pusher thrust, then rpm search
        speed_ref=float(target.get('speed_mps',Va)); state.speed_integral=clamp(state.speed_integral+(speed_ref-Va)*dt,-30,30)
        Treq=max(0,float(cc['speed_kp_N_per_mps'])*(speed_ref-Va)+float(cc['speed_ki_N_per_m'])*state.speed_integral+max(0,-ar.force_body[0]))
        rpm_cmd=0.0; besterr=1e9
        for rpm in np.linspace(0,float(cfg['pusher']['max_rpm']),101):
            T,_=self.m.pusher_point(Va,float(rpm)); err=abs(T-Treq)
            if err<besterr:besterr=err;rpm_cmd=float(rpm)
        return Controls(thrusts,rpm_cmd,surface,Mdes,Mrot,Msurf,resid)


def target_from_phase5_rows(rows:Sequence[Mapping[str,float]],t:float,final_speed_override:Optional[float]=None)->Dict[str,float]:
    if t<=rows[0]['time_s']: r=rows[0]
    elif t>=rows[-1]['time_s']:
        r=dict(rows[-1])
        if final_speed_override is not None: r['vx_mps']=float(final_speed_override)
        r['vz_mps']=0.0
    else:
        # binary-ish linear search sufficient for <1000 rows
        j=1
        while j<len(rows) and rows[j]['time_s']<t:j+=1
        a,b=rows[j-1],rows[j]; f=(t-a['time_s'])/max(b['time_s']-a['time_s'],EPS)
        r={k:float(a[k])+(float(b[k])-float(a[k]))*f for k in ['vx_mps','pitch_deg','altitude_m','vz_mps']}
        return {'speed_mps':r['vx_mps'],'pitch_deg':r['pitch_deg'],'altitude_m':r['altitude_m'],'vertical_speed_mps':r['vz_mps'],'roll_deg':0,'yaw_deg':0}
    return {'speed_mps':float(r['vx_mps']),'pitch_deg':float(r['pitch_deg']),'altitude_m':float(r['altitude_m']),'vertical_speed_mps':float(r['vz_mps']),'roll_deg':0,'yaw_deg':0}


def make_sample(model:Model,state:State,ctrl:Controls,target:Mapping[str,float],wind:np.ndarray)->Sample:
    ar=model.aero(state,wind); phi,theta,psi=euler_from_quat(state.quat_bn); alt=-state.pos_ned[2]
    cs=model.cfg['control_surfaces']; slims=np.radians([cs['aileron_limit_deg'],cs['elevator_limit_deg'],cs['rudder_limit_deg']]); surfutil=float(np.max(np.abs(state.surfaces)/slims))
    screen_clmax=float(model.cfg['aerodynamics'].get('stall_screen_CLmax', model.cfg['aerodynamics']['CL_max']))
    clutil=float(ar.CL/max(screen_clmax,EPS))
    return Sample(state.t,float(state.pos_ned[0]),float(state.pos_ned[1]),float(alt),*map(float,state.vel_body),ar.Va,degrees(phi),degrees(theta),degrees(psi),*map(degrees,state.omega_body),degrees(ar.alpha),degrees(ar.beta),float(np.sum(state.lift_thrusts)),float(state.pusher_rpm),*map(float,state.lift_thrusts),degrees(state.surfaces[0]),degrees(state.surfaces[1]),degrees(state.surfaces[2]),float(np.max(state.lift_thrusts)/model.tmax),surfutil,float(target.get('pitch_deg',0)),float(target.get('speed_mps',0)),float(target.get('altitude_m',alt)),degrees(theta)-float(target.get('pitch_deg',0)),alt-float(target.get('altitude_m',alt)),ctrl.rotor_allocation_residual,float(ar.CL),clutil)


def simulate_tracking(model:Model,rows:Sequence[Mapping[str,float]],wind_fn=None,initial_roll_deg:float=0,initial_yaw_deg:float=0,final_speed_override:Optional[float]=20.0,max_duration_s:Optional[float]=None)->List[Sample]:
    dt=float(model.cfg['simulation']['dt_s']); tmax=float(model.cfg['simulation']['forward_transition_max_s']) if max_duration_s is None else float(max_duration_s)
    q0=quat_from_euler(radians(initial_roll_deg),radians(float(rows[0]['pitch_deg'])),radians(initial_yaw_deg))
    earth_v=np.array([max(.01,float(rows[0]['vx_mps'])),0,-float(rows[0]['vz_mps'])],dtype=float)
    body_v=rot_body_to_ned(q0).T@earth_v
    if float(rows[0]['vx_mps'])<5.0:
        initial_lift,_=model.rotor_allocate(model.m*G,np.zeros(3))
    else:
        initial_lift=np.zeros(4)
    st=State(0,np.array([0,0,-float(rows[0]['altitude_m'])],dtype=float),body_v,q0,np.zeros(3),initial_lift.copy(),0,np.zeros(3))
    ctl=Controller(model); out=[]
    while st.t<=tmax+EPS:
        target=target_from_phase5_rows(rows,st.t,final_speed_override=final_speed_override); wind=np.zeros(3) if wind_fn is None else np.asarray(wind_fn(st.t),dtype=float)
        c=ctl.command(st,target,wind,dt); model.actuator_update(st,c,dt); out.append(make_sample(model,st,c,target,wind)); model.integrate(st,dt,wind)
        if not np.all(np.isfinite(np.concatenate([st.pos_ned,st.vel_body,st.quat_bn,st.omega_body]))):break
        phi,theta,_=euler_from_quat(st.quat_bn)
        if abs(degrees(phi))>60 or abs(degrees(theta))>60 or -st.pos_ned[2]<0:break
    return out


def simulate_cruise_gust(model:Model,speed_mps:float=20.0,stabilized:bool=True,gust_ned_mps:Optional[Sequence[float]]=None)->List[Sample]:
    trim=model.trim_cruise(speed_mps); dt=float(model.cfg['simulation']['dt_s']); dur=float(model.cfg['simulation']['cruise_disturbance_duration_s']); sim=model.cfg['simulation']
    gust_vec=np.asarray(sim['wind_gust_ned_mps'] if gust_ned_mps is None else gust_ned_mps,dtype=float)
    if gust_vec.shape != (3,):
        raise ValueError('gust_ned_mps must contain exactly three NED components')
    alpha=radians(trim['alpha_deg']); q0=quat_from_euler(0,alpha,0)
    rpm0=0.0; err=1e9
    for rpm in np.linspace(0,float(model.cfg['pusher']['max_rpm']),161):
        T,_=model.pusher_point(speed_mps,float(rpm)); e=abs(T-trim['pusher_thrust_N'])
        if e<err: err=e; rpm0=float(rpm)
    st=State(0,np.array([0,0,-30.0]),np.array([speed_mps*cos(alpha),0,speed_mps*sin(alpha)]),q0,np.zeros(3),np.zeros(4),rpm0,np.array([0,radians(trim['elevator_deg']),0]))
    ctl=Controller(model); out=[]
    while st.t<=dur+EPS:
        wind=np.zeros(3)
        if float(sim['wind_gust_start_s'])<=st.t<=float(sim['wind_gust_start_s'])+float(sim['wind_gust_duration_s']): wind=gust_vec.copy()
        target={'speed_mps':speed_mps,'pitch_deg':trim['alpha_deg'],'altitude_m':30.0,'vertical_speed_mps':0,'roll_deg':0,'yaw_deg':0}
        c=ctl.command(st,target,wind,dt) if stabilized else Controls(np.zeros(4),st.pusher_rpm,st.surfaces.copy(),np.zeros(3),np.zeros(3),np.zeros(3),0)
        if stabilized: model.actuator_update(st,c,dt)
        out.append(make_sample(model,st,c,target,wind)); model.integrate(st,dt,wind)
        phi,theta,_=euler_from_quat(st.quat_bn)
        if abs(degrees(phi))>70 or abs(degrees(theta))>70:break
    return out


def summarize_samples(samples:Sequence[Sample],model:Model)->Dict[str,Any]:
    if not samples:return {'success':False,'reason':'no_samples'}
    g=model.cfg['gates']; alts=[s.altitude_m for s in samples]
    return {
        'samples':len(samples),'duration_s':samples[-1].time_s,'final_speed_mps':samples[-1].airspeed_mps,
        'maximum_abs_roll_deg':max(abs(s.roll_deg) for s in samples),'maximum_abs_yaw_deg':max(abs(s.yaw_deg) for s in samples),
        'maximum_abs_pitch_tracking_error_deg':max(abs(s.pitch_tracking_error_deg) for s in samples),
        'maximum_abs_beta_deg':max(abs(s.beta_deg) for s in samples),'maximum_abs_alpha_deg':max(abs(s.alpha_deg) for s in samples),
        'minimum_altitude_m':min(alts),'altitude_loss_m':max(0,samples[0].altitude_m-min(alts)),
        'maximum_surface_utilization':max(s.max_surface_utilization for s in samples),'maximum_rotor_utilization':max(s.max_rotor_utilization for s in samples),
        'maximum_positive_CL':max(s.CL for s in samples),'minimum_CL':min(s.CL for s in samples),
        'maximum_clmax_utilization':max(s.clmax_utilization for s in samples),
        'maximum_rotor_allocation_residual':max(s.rotor_allocation_residual for s in samples),
        'final_roll_deg':samples[-1].roll_deg,'final_pitch_deg':samples[-1].pitch_deg,'final_yaw_deg':samples[-1].yaw_deg,
    }
