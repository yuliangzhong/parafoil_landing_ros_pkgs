import numpy as np
from numpy import dot
from random import randint
from math import pi, sin, cos, sqrt, atan, asin

# help functions
def ImIiCompute(a, b, c, t):
    a_bar = a/b
    t_bar = t/c
    AR = b/c
    A = 0.666 * (1 + 8/3*a_bar**2) * t**2 * b
    B = 0.267 * (t**2 + 2*a**2*(1-t_bar**2))*c
    C = 0.785 * sqrt(1 + 2*a_bar**2*(1-t_bar**2))*AR/(1+AR)*b*c**2
    D = 0.055 * AR/(1+AR)*b**3*c**2
    E = 0.0308 * AR/(1+AR)*(1 + pi/6*(1+AR)*AR*a_bar**2*t_bar**2)*b*c**4
    F = 0.0555 * (1+8*a_bar**2)*t**2*b**3
    Im = np.diag([A, B, C])
    Ii = np.diag([D, E, F])
    return Im, Ii

def rpy2matrix3(rpy3x1):
    rpy = rpy3x1.reshape(-1)
    phi, theta, psi = rpy[0], rpy[1], rpy[2]
    C = np.array([[cos(theta)*cos(psi),    sin(phi)*sin(theta)*cos(psi)-cos(theta)*sin(psi),   cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi)], \
                  [cos(theta)*sin(psi),    sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi),     cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi)], \
                  [-sin(theta),            sin(phi)*cos(theta),                                cos(phi)*cos(theta)]])
    return C

def cross_product(a, b):
    return np.cross(a.reshape(-1), b.reshape(-1)).reshape(-1,1)

def skew(x):
    x = x.reshape(-1)
    return np.array([[0, -x[2], x[1]],
                     [x[2], 0, -x[0]],
                     [-x[1], x[0], 0]])

# sim constant
rho = 1.29 # kg/m3

# system constant
system_mass = 2.2
system_inertia = np.array([[1.68, 0, 0.09], [0, 0.80, 0], [0.09, 0, 0.32]])

parafoil_ref_area = 1.5 # m2
parafoil_ref_span = 1.88 # m
parafoil_ref_length = 0.80 # m, chord
parafoil_arc_h = 0.1 # m
parafoil_thickness = 0.075 # m, thickness of canopy

S_pd = 0.06 # m2 payload reference area 

C_BP = np.eye(3) # from body frame B to parafoil aero frame P
B_r_BP = np.array([0, 0, -1.2]).reshape(-1,1)

C_BW = np.eye(3) # from body frame B to payload frame W
B_r_BW = np.array([0, 0, 0.1]).reshape(-1,1)

Im, Ii = ImIiCompute(parafoil_arc_h, parafoil_ref_span, parafoil_ref_length, parafoil_thickness)

c_L0 = 0.24 # lift force coefficient
c_La = 2.14
c_Lds = 0.2783

c_D0 = 0.12
c_Da2 = 0.33
c_Dds = 0.43 # large enough to enable gliding ratio control 17.5-26.3

c_Yb = 0.23

c_lp = -0.84 # negative
c_lda = -0.005 # negative
c_m0 = 0.1 
c_ma = -0.72
c_mq = -1.49 # negative
c_nr = -0.27 # negative
c_nda = 0.0115 # positive

c_Dpd = 3.2 # payload drag coefficient

cL = [c_L0, c_La, c_Lds] # lift force coefficients
cD = [c_D0, c_Da2, c_Dds] # drag force coefficients
cM = [c_lp, c_lda, c_m0, c_ma, c_mq, c_nr, c_nda] # moment coefficients



class Simulator(object):
    def __init__(self):
        self.cnt = 1
        self.rand_int = 100
        self.current_wind = np.zeros((3,1))

        self.pos = np.array([0, 0, -50]).reshape(-1,1)
        self.rpy = np.array([0, 0.006, -10/180*pi]).reshape(-1,1)
        self.uvw = np.array([3.819, -0.673, 1.62]).reshape(-1,1)
        self.pqr = np.zeros((3,1))

        self.delta_l = 0.5 # [0,1]
        self.delta_r = 0.5 # [0,1]
        
    # def action_callback(self, msg):
    #     self.delta_r = max(0, min(1, msg.data.x))
    #     self.delta_l = max(0, min(1, msg.data.y))
    
    def wind_generator(self):
        if self.cnt % self.rand_int > self.rand_int - 5:
            self.current_wind = np.random.multivariate_normal([0,0,0], [[1,0,0],[0,1,0],[0,0,0]], 1).reshape(-1,1)
            self.rand_int = randint(100, 1000)
        if self.cnt > 1e9:
            self.cnt = 1

    def sim_step(self):
        self.cnt += 1

        B_V_w = dot(rpy2matrix3(self.rpy).T, self.current_wind)
        B_v_IB_tilde = self.uvw - B_V_w
        P_v_IP_tilde = dot(C_BP.T,B_v_IB_tilde + cross_product(self.pqr, B_r_BP))
        # cL, cD, b, c, C_BP, parafoil_ref_area, cM, B_r_BP, c_Dpd, S_pd, B_r_BW, C_BW, cYb
        delta_s = (self.delta_l + self.delta_r) / 2
        delta_a = self.delta_r - self.delta_l

        u, v, w = P_v_IP_tilde[0], P_v_IP_tilde[1], P_v_IP_tilde[2]
        alpha = atan(w/u) # angle of attack
        V_P = sqrt(u**2+v**2+w**2)
        beta = asin(v/V_P) # angle of sideslip
        C_PF = np.array([[cos(alpha)*cos(beta), -cos(alpha)*sin(beta), -sin(alpha)], \
                         [sin(beta),            cos(beta),             0          ], \
                         [sin(alpha)*cos(beta), -sin(alpha)*sin(beta), cos(alpha)]])

        P_w_IP_skew = dot(C_BP.T, dot(skew(self.pqr), C_BP))
        p, q, r = P_w_IP_skew[2,1], P_w_IP_skew[0,2], P_w_IP_skew[1,0]

        tmp = -np.array([cD[0] + alpha**2*cD[1] + delta_s*cD[2], beta * c_Yb, cL[0] + alpha*cL[1] + delta_s*cL[2]]).reshape(-1,1)
        P_F_A = 0.5 * rho * parafoil_ref_area * V_P**2 * dot(C_PF, tmp)

        c_l = dot(np.array([(parafoil_ref_span/2/V_P)*p, delta_a]), np.array([cM[0], cM[1]]).reshape(-1,1))
        c_m = dot(np.array([1, alpha, (parafoil_ref_length/2/V_P)*q]), np.array([cM[2], cM[3], cM[4]]).reshape(-1,1))
        c_n = dot(np.array([(parafoil_ref_span/2/V_P)*r, delta_a]), np.array([cM[5], cM[6]]).reshape(-1,1))
        P_M_A = 0.5 * rho * parafoil_ref_area * V_P**2 * np.array([parafoil_ref_span*c_l, parafoil_ref_length*c_m, parafoil_ref_span*c_n]).reshape(-1,1)
    
        W_v_IW_tilde = dot(C_BW.T, (B_v_IB_tilde + cross_product(self.pqr, B_r_BW)))
        W_F_pd = -0.5*rho*S_pd*c_Dpd*np.linalg.norm(W_v_IW_tilde)*W_v_IW_tilde

        B_F = dot(C_BP, P_F_A) + dot(C_BW, W_F_pd)
        B_M = dot(C_BP, P_M_A) + cross_product(B_r_BP, P_F_A) + cross_product(B_r_BW, W_F_pd)



    def sensor_pub(self):
        pass
        # add sensor noise, publish it


if __name__ == '__main__':
    sim = Simulator()
    sim.sim_step()