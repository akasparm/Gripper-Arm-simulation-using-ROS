import numpy as np
from roboticstoolbox import DHRobot, RevoluteDH
from spatialmath import SE3
import roboticstoolbox as rtb


class UR5(DHRobot):

    def __init__(self, symbolic=False):

        if symbolic:
            import spatialmath.base.symbolic as sym

            zero = sym.zero()
            pi = sym.pi()
        else:
            from math import pi

            zero = 0.0

        deg = pi / 180
        inch = 0.0254

        # robot length values (metres)
        a = [0, -0.42500, -0.39225, 0, 0, 0]
        d = [0.089459, 0, 0, 0.10915, 0.09465, 0.0823]

        alpha = [pi / 2, zero, zero, pi / 2, -pi / 2, zero]

        mass = [3.7000, 8.3930, 2.33, 1.2190, 1.2190, 0.1897]

        center_of_mass = [
            [0, -0.02561, 0.00193],
            [0.2125, 0, 0.11336],
            [0.15, 0, 0.0265],
            [0, -0.0018, 0.01634],
            [0, -0.0018, 0.01634],
            [0, 0, -0.001159],
        ]

        links = []

        for j in range(6):
            link = RevoluteDH(
                d=d[j], a=a[j], alpha=alpha[j], m=mass[j], r=center_of_mass[j], G=1
            )
            links.append(link)

        super().__init__(
            links,
            name="UR5",
            manufacturer="Universal Robotics",
            keywords=("dynamics", "symbolic"),
            symbolic=symbolic,
        )

        self.qr = np.array([180, 0, 0, 0, 90, 0]) * deg
        self.qz = np.zeros(6)

        self.addconfiguration("qr", self.qr)
        self.addconfiguration("qz", self.qz)



if __name__ == "__main__":

    ur5 = UR5(symbolic=False)
    print(ur5)

    # compute the forward kinematics 

    Te = ur5.fkine(ur5.qr) 
    print("Forward Kinematics Validation:")
    print(Te)

