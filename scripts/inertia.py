import math


class IMat:
    def __init__(self):
        self.ixx = 0
        self.ixy = 0
        self.ixz = 0
        self.iyy = 0
        self.iyz = 0
        self.izz = 0

    def __str__(self):
        return '<inertia ixx="{}" ixy="{}" ixz="{}" iyy="{}" iyz="{}" izz="{}" />'.format(self.ixx, self.ixy, self.ixz,
                                                                                         self.iyy, self.iyz, self.izz)


print "WELCOME to the urdf inertia tool, for those too lazy to do it themselves."
print "Written by M2 with the help of wikipedia"

while True:
    print "Choose body type:"
    print "[b] Box (solid cuboid)"
    print "[c] Cylinder (solid)"
    print "[s] Sphere (solid)"
    t = raw_input("> ")
    imat = IMat()
    m = float(raw_input("mass: "))
    if t == "b":
        x = float(raw_input("size.x: "))
        y = float(raw_input("size.y: "))
        z = float(raw_input("size.z: "))

        imat.ixx = 1 / 12.0 * m * (y ** 2 + z ** 2)
        imat.iyy = 1 / 12.0 * m * (x ** 2 + z ** 2)
        imat.izz = 1 / 12.0 * m * (y ** 2 + x ** 2)
    elif t == "c":
        r = float(raw_input("radius: "))
        h = float(raw_input("height: "))
        a = raw_input("height.axis (x/y/z): ")
        if a.lower() == "z":
            imat.ixx = imat.iyy = 1/12.0 * m * (3 * r**2 + h**2)
            imat.izz = 1/2.0 * m * r**2
        elif a.lower() == "y":
            imat.ixx = imat.izz = 1 / 12.0 * m * (3 * r ** 2 + h ** 2)
            imat.iyy = 1 / 2.0 * m * r ** 2
        elif a.lower() == "x":
            imat.iyy = imat.izz = 1 / 12.0 * m * (3 * r ** 2 + h ** 2)
            imat.ixx = 1 / 2.0 * m * r ** 2
    else:
        print "not done yet"
    print "Put this in your <inertial> tag:"
    print imat

    yn = raw_input("Do another? [yN] ")
    if yn == "": break
    if yn.upper() == "Y": continue
    break
