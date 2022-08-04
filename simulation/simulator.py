import numpy as np
from numpy import dot, float64
from random import randint
from math import pi, sin, cos, tan, sqrt, atan, atan2, asin
import matplotlib.pyplot as plt

##### Definition #####
# rpy: roll, pitch, yaw. Sequence: yaw-pitch-roll
# q: quaternion, [[q0, q1, q2, q3]] = [q0, q_h]

# help functions
def Sign(x):
    if x >= 0:
        return 1
    else:
        return -1

def skew(x):
    x = x.reshape(-1)
    return np.array([[0, -x[2], x[1]],
                     [x[2], 0, -x[0]],
                     [-x[1], x[0], 0]])

def rpy2matrix3(rpy3x1):
    rpy = rpy3x1.reshape(-1)
    phi, theta, psi = rpy[0], rpy[1], rpy[2]
    C = np.array([[cos(theta)*cos(psi),    sin(phi)*sin(theta)*cos(psi)-cos(theta)*sin(psi),   cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi)], \
                  [cos(theta)*sin(psi),    sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi),     cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi)], \
                  [-sin(theta),            sin(phi)*cos(theta),                                cos(phi)*cos(theta)]])
    return C

def matrix2quat(C):
    quat = 0.5* np.array([[sqrt(C[0,0] + C[1,1] + C[2,2] + 1)],\
                          [Sign(C[2,1]-C[1,2])*sqrt(C[0,0] - C[1,1] - C[2,2] +1)],\
                          [Sign(C[0,2]-C[2,0])*sqrt(C[1,1] - C[2,2] - C[0,0] +1)],\
                          [Sign(C[1,0]-C[0,1])*sqrt(C[2,2] - C[0,0] - C[1,1] +1)]])
    n = np.linalg.norm(quat)
    return quat / n

def quat_product(q1, q2):
    q_10 = q1[0][0]
    q_1h = q1[1:4,:].reshape(-1,1)
    q_20 = q2[0][0]
    q_2h = q2[1:4,:].reshape(-1,1)
    q0 = q_10*q_20 - dot(q_1h.T, q_2h)
    q_h = q_10*q_2h+q_20*q_1h+cross_product(q_1h, q_2h)
    return np.vstack((q0, q_h))

def quat2matrix(q):
    n = np.linalg.norm(q)
    q = q / n
    q0 = q[0][0]
    q_h = q[1:4,:].reshape(-1,1)
    return (2*q0**2-1)*np.eye(3) + 2*q0*skew(q_h) + 2*dot(q_h, q_h.T)

def matrix2rpy(C):
    return np.array([[atan2(C[2,1],C[2,2])],\
                     [atan2(-C[2,0], np.linalg.norm([C[2,1], C[2,2]]))],\
                     [atan2(C[1,0], C[0,0])]])

def cross_product(a, b):
    return np.cross(a.reshape(-1), b.reshape(-1)).reshape(-1,1)

# sim constant
dT = 0.05
rho = 1.29 # kg/m3
gravity_acc = + 9.81 # z-down

# system constant
system_mass = 2.2
system_inertia = np.array([[1.68, 0, 0.09], [0, 0.80, 0], [0.09, 0, 0.32]]) # in body frame

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
        self.current_wind = np.zeros((3,1), dtype=float64)

        # position, quaternion, velocity, angular velocity in the initial(ground) frame
        self.pos = np.array([-10, -10, -50], dtype=float64).reshape(-1,1)
        self.quat = matrix2quat(rpy2matrix3(np.array([0, 0.006, -10/180*pi], dtype=float64).reshape(-1,1)))
        self.vel = dot(quat2matrix(self.quat), np.array([3.819, -0.673, 1.62], dtype=float64).reshape(-1,1))
        self.ang_vel = np.zeros((3,1), dtype=float64)
        # self.uvw = np.array([3.819, -0.673, 1.62], dtype=float64).reshape(-1,1)
        # self.pqr = np.zeros((3,1), dtype=float64)
        # self.rpy = np.array([0, 0.006, -10/180*pi], dtype=float64).reshape(-1,1)

        self.delta_l = 0.5 # [0,1]
        self.delta_r = 0.5 # [0,1]
    
    def print_status(self):
        print("-----")
        print("pos: ", self.pos.reshape(-1))
        print("vel: ", self.vel.reshape(-1))
        print("quat: ", self.quat.reshape(-1))
        print("omega: ", self.ang_vel.reshape(-1))
        # print("rpy: ", self.rpy.reshape(-1))
        # print("uvw: ", self.uvw.reshape(-1))
        # print("pqr: ", self.pqr.reshape(-1))
        print("count: ", self.cnt)
        
    # def action_callback(self, msg):
    #     self.delta_r = max(0, min(1, msg.data.x))
    #     self.delta_l = max(0, min(1, msg.data.y))
    
    # def wind_generator(self):
    #     if self.cnt % self.rand_int > self.rand_int - 5:
    #         self.current_wind = np.random.multivariate_normal([0,0,0], [[1,0,0],[0,1,0],[0,0,0]], 1).reshape(-1,1)
    #         self.rand_int = randint(100, 1000)
    #     if self.cnt > 1e9:
    #         self.cnt = 1

    def sim_forward(self):
        self.cnt += 1

        ##### calculate total force and torque (without apparent mass) #####

        # B_V_w = dot(rpy2matrix3(self.rpy).T, self.current_wind)
        # B_v_IB_tilde = self.uvw - B_V_w
        # P_v_IP_tilde = dot(C_BP.T,B_v_IB_tilde + cross_product(self.pqr, B_r_BP))
        # delta_s = (self.delta_l + self.delta_r) / 2
        # delta_a = self.delta_r - self.delta_l

        # u, v, w = P_v_IP_tilde[0], P_v_IP_tilde[1], P_v_IP_tilde[2]
        # alpha = atan(w/u) # angle of attack
        # V_P = sqrt(u**2+v**2+w**2)
        # beta = asin(v/V_P) # angle of sideslip
        # C_PF = np.array([[cos(alpha)*cos(beta), -cos(alpha)*sin(beta), -sin(alpha)], \
        #                  [sin(beta),            cos(beta),             0          ], \
        #                  [sin(alpha)*cos(beta), -sin(alpha)*sin(beta), cos(alpha)]])

        # P_w_IP_skew = dot(C_BP.T, dot(skew(self.pqr), C_BP))
        # p, q, r = P_w_IP_skew[2,1], P_w_IP_skew[0,2], P_w_IP_skew[1,0]

        # tmp = -np.array([cD[0] + alpha**2*cD[1] + delta_s*cD[2], beta * c_Yb, cL[0] + alpha*cL[1] + delta_s*cL[2]]).reshape(-1,1)
        # P_F_A = 0.5 * rho * parafoil_ref_area * V_P**2 * dot(C_PF, tmp)

        # c_l = dot(np.array([(parafoil_ref_span/2/V_P)*p, delta_a]), np.array([cM[0], cM[1]]).reshape(-1,1))
        # c_m = dot(np.array([1, alpha, (parafoil_ref_length/2/V_P)*q]), np.array([cM[2], cM[3], cM[4]]).reshape(-1,1))
        # c_n = dot(np.array([(parafoil_ref_span/2/V_P)*r, delta_a]), np.array([cM[5], cM[6]]).reshape(-1,1))
        # P_M_A = 0.5 * rho * parafoil_ref_area * V_P**2 * np.array([parafoil_ref_span*c_l, parafoil_ref_length*c_m, parafoil_ref_span*c_n]).reshape(-1,1)
    
        # W_v_IW_tilde = dot(C_BW.T, (B_v_IB_tilde + cross_product(self.pqr, B_r_BW)))
        # W_F_pd = -0.5*rho*S_pd*c_Dpd*np.linalg.norm(W_v_IW_tilde)*W_v_IW_tilde

        # B_F_a = dot(C_BP, P_F_A) + dot(C_BW, W_F_pd)
        # B_M_a = dot(C_BP, P_M_A) + cross_product(B_r_BP, P_F_A) + cross_product(B_r_BW, W_F_pd)

        # B_G = system_mass*gravity_acc*np.array([-sin(self.rpy[1]), sin(self.rpy[0])*cos(self.rpy[1]), cos(self.rpy[0])*cos(self.rpy[1])]).reshape(-1,1)

        # B_F = B_F_a + B_G
        # B_M = B_M_a

        # for debugging
        B_F = np.array([0.2086225,   2.03626621, -1.24894524]).reshape(-1,1)
        B_M = np.array([1.98817463, -5.57137594,  0.        ]).reshape(-1,1)
        
        C_IB = quat2matrix(self.quat)
        I_ground = dot(dot(C_IB, system_inertia), C_IB.T)
        ##### calculate system states by symplectic Euler method #####
        vel_new = self.vel + dT/system_mass*dot(C_IB, B_F)

        #     rb->state.orientation = updateRotationGivenAngularVelocity(rb->state.orientation,w_new,dt);
        #     rb->state.velocity = v_new;
        #     rb->state.angularVelocity = w_new;
        self.pos += dT*vel_new
        nm = np.linalg.norm(self.ang_vel)
        if nm > 1e-10:
            # print("in cos:",dT*nm/2)
            q_tmp0 = cos(dT*nm/2)
            q_tmph = sin(dT*nm/2)/nm*self.ang_vel
            q_tmp = np.vstack((np.array([[q_tmp0]]), q_tmph))
            self.quat = quat_product(q_tmp, self.quat)
        else:
            self.quat = self.quat
        
        ang_vel_new = self.ang_vel + dT*dot(np.linalg.inv(I_ground),(dot(C_IB, B_M) - cross_product(self.ang_vel, dot(I_ground, self.ang_vel))))

        self.vel = vel_new
        self.ang_vel = ang_vel_new
        # ground_vel_old = dot(rpy2matrix3(self.rpy), self.uvw)
        # ground_vel_new = ground_vel_old + dT * dot(rpy2matrix3(self.rpy), B_F) / system_mass
        # self.uvw = dot(rpy2matrix3(self.rpy).T, ground_vel_new)


    def sensor_pub(self):
        pass
        # add sensor noise, publish it


if __name__ == '__main__':
    sim = Simulator()
    for i in range(6000):
        
        sim.sim_forward()
        # plt.plot([count-1, count], [last_h, h], color='red')
        # plt.pause(0.001)
    sim.print_status()