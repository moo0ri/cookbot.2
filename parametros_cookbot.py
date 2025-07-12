#PARÂMETROSROBÔ
from roboticstoolbox import ERobot, Link, ET
import numpy as np
# Definindo os parâmetros do robô Cookbot
a2 = 0.6
a3 = 0.2
m0 = 0.816
m1 = 1.094
m2 = 1.958
m3 = 0.941
g = 9.81
I0 = np.array([[4.653, -5646.462e-06, 78206.614e-06],
                [-5646.462e-06, 4.743, 3671.124e-06],
                [78206.614e-06, 3671.124e-06, 3.209]])
I1 = np.array([[1.206, -5684.267e-07, -4.323e-02],
                [-5684.267e-07, 1.217, -17964.573e-07],
                [-4.323e-02, -17964.573e-07, 1.835e-06]])
I2 = np.array([[0.005, 4.019e-05, -0.001],
                [4.019e-05, 0.099, 5.948e-06],
                [-0.001,  5.948e-06, 0.101]])
I3 = np.array([[ 0.01353,    -0.0002274, 0.00001642],
               [-0.0002274,   0.009331,   0.00001067],
               [ 0.00001642,  0.00001067, 0.01379]])
# Definindo o manipulador robótico
link0 = Link(ET.tz(qlim=[0, 0.170]), r = [3.222e-3, 0.155e-3, 46.056e-3], m = m0, I = I0, Tc = 0.0107, Jm = 2.4e-07, B = 2e-04)
link1 = Link(ET.tz(0.300) * ET.Rz(qlim=[-np.pi, np.pi]), r = [0.002836, 0.000116, 0.110482], m = m1, I = I1, Tc = 0.0107, Jm = 2.4e-07, B = 2e-04)
link2 = Link(ET.tx(a2) * ET.tx(a3) * ET.Rx(qlim=[-np.pi, np.pi]), r = [0.266, -6.265e-5, 0.034], m = m2, I = I2, Tc = 0.0107, Jm = 2.4e-07, B = 2e-04)
link3 = Link(ET.Ry(np.pi/6) * ET.tz(-0.100) * ET.tx(0.100) * ET.tz(0.100), r = [-0.000174, 0.043658, 0.000155], m = m2, I = I2, Tc = 0.0107, Jm = 2.4e-07, B = 2e-04)

cookbot = ERobot([link0, link1, link2, link3], name="Cookbot")