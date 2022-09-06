import yaml
import numpy as np
from math import cos, sin, pi
from sensor_msgs.msg import NavSatFix

R = 6371000 # m, radius of earth

params = {}
with open("config.yaml", "r") as stream:
    try:
        params = yaml.safe_load(stream)
    except yaml.YAMLError as exc:
        print(exc)

# load system parameters
a = params['a']
k = params['k']
Ts = params['Ts']

# load init conditions
pos = np.array([params['x0'], params['y0']]).reshape(-1,1)
psi_s = np.array([params['psi0'], params['psi_dot0']]).reshape(-1,1)
Vh = params['Vh']
self.origin_lat = 47.35504 # degree
self.origin_lon = 8.51982 # degree


# forward Euler
Ac = np.array([[0, 1], [0, 1/a]])
Bc = np.array([0, k/a]).reshape(-1,1)
Ad = np.eye(2) + Ts*Ac
Bd = Ts*Bc

# loop
da = 0 # from msg
tmp_psi_s = psi_s
psi_s = np.dot(Ad, psi_s) + Bd*da
avg_psi = (tmp_psi_s[0][0] + psi_s[0][0])/2
pos += Ts*Vh*np.array([cos(avg_psi), sin(avg_psi)]).reshape(-1,1)

cvt_msg = NavSatFix()
cvt_msg.header = msg.header
cvt_msg.latitude = float(pos[0][0] / R / pi * 180 + self.origin_lat)
cvt_msg.longitude = float(pos[0][1] / (R * cos(self.origin_lat / 180 * pi)) / pi * 180 + self.origin_lon)
# self.pub_gps.publish(cvt_msg)





