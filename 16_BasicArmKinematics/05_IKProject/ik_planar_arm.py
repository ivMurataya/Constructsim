import math 
from math import pi



class ComputeIk:
    def __init__(self,DH_parameters):
        self.r1 = DH_parameters['r1']
        self.r2 = DH_parameters['r2']
        self.r3 = DH_parameters['r3']             

    def compute_ik(self, Pee_x, Pee_y, chi, elbow_config='down'):
        # Step 1: Compute wrist position
        U = self.r3 * math.cos(chi)
        W = self.r3 * math.sin(chi)
        Px = Pee_x - U
        Py = Pee_y - W

        # Step 2: Compute G using the law of cosines
        num = Px**2 + Py**2 - self.r1**2 - self.r2**2
        den = 2 * self.r1 * self.r2
        G = num / den

        # Check if G is within valid range
        if abs(G) > 1:
            return ([], False)

        # Step 3: Compute theta_2
        if elbow_config == 'up':
            theta_2 = math.atan2(-math.sqrt(1 - G**2), G)
        else:
            theta_2 = math.atan2(math.sqrt(1 - G**2), G)


        # Step 4: Compute theta_1
        k1 = self.r1 + self.r2 * math.cos(theta_2)
        k2 = self.r2 * math.sin(theta_2)
        theta_1 = math.atan2(Py, Px) - math.atan2(k2, k1)

        # Step 5: Compute theta 3
        theta_3 = chi - theta_1 - theta_2
        
        # theta_1 = math.radians(theta_1)
        # theta_2 = math.radians(theta_2)
        # theta_3 = math.radians(theta_3)
        return ([theta_1,theta_2,theta_3],True)




def calculate_ik(Pee_x, Pee_y, chi, DH_parameters, elbow_config):
    ik_solver = ComputeIk(DH_parameters)
    return ik_solver.compute_ik(Pee_x, Pee_y, chi, elbow_config)

DH = {'r1': 1.0, 'r2': 1.0, 'r3': 1.0}
x = 0.5
y = 0.5
phi = -pi/2  # 45 degrees

angles, valid = calculate_ik(x, y, phi, DH, elbow_config='up')

if valid:
    print("Theta angles (in degrees):", [ angles])
else:
    print("No valid solution found.")

