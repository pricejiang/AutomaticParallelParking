import numpy as np
import scipy
import matplotlib.pyplot as plt
import matplotlib.patches as patches

# Define constants based on tech spec's of GEM car
# e = 1.753
# length = 2.62
# p = (length-e)/2
# R = 3.81
# betaMax = np.arcsin(e/R)
# betaMin = -betaMax
# w = 1.41


# Ri_min = (e/np.tan(betaMax)) - w/2
# Re_min = np.sqrt( pow(np.sqrt(R*R - e*e) + w/2, 2) + (e+p)*(e+p))
# Lmin = p + np.sqrt(Re_min*Re_min - Ri_min*Ri_min)

# R_min_ = Ri_min + w/2

# Assume parking ending position at (0, 0)
# xe, ye = 0, 0

def calcParking(xe, ys, dy):
    e = 1.753
    length = 2.62
    p = (length-e)/2
    R = 3.81
    betaMax = np.arcsin(e/R)
    betaMin = -betaMax
    w = 1.41


    Ri_min = (e/np.tan(betaMax)) - w/2
    Re_min = np.sqrt( pow(np.sqrt(R*R - e*e) + w/2, 2) + (e+p)*(e+p))
    Lmin = p + np.sqrt(Re_min*Re_min - Ri_min*Ri_min)

    R_min_ = Ri_min + w/2
    
    xc1 = xe 
    # print(yc1, xc1)
    # Assume start position of y
    # y is car_width + 0.5 meters away 
    # xs = Lmin + p*2
    # ys = ye + w + 0.5

    ye = ys - w - dy # XXX change 5 to a variable
    yc2 = ys - R_min_ 
    yc1 = ye + R_min_
    yt = (yc1 + yc2)/2

    xt = xc1 + np.sqrt(R_min_*R_min_ - (yt-yc1)*(yt-yc1))

    xs = xc2 = 2*xt - xc1

    print("C1: ", xc1, yc1)
    print("C2: ", xc2, yc2)
    print("R~min: ", R_min_)
    print("Ri min: ", Ri_min)
    print("Re min: ", Re_min)

    x = [xs, xe, xc1, xc2, xt]
    y = [ys, ye, yc1, yc2, yt]
    c1 = plt.Circle((xc1, yc1), R_min_, color='r')
    c2 = plt.Circle((xc2, yc2), R_min_, color='b')

    xp = []
    yp = []

    x = np.arange(xc1, xt, 0.1)
    y = -np.sqrt(R_min_*R_min_ - (x-xc1)*(x-xc1)) + yc1
    xp += list(x) 
    yp += list(y)
    x = np.arange(xt, xc2, 0.1)
    y = np.sqrt(R_min_*R_min_ - (x-xc2)*(x-xc2)) + yc2
    xp += list(x)
    yp += list(y)

    print(zip(xp, yp))

    path = zip(xp, yp)

    path.reverse()

    xs, ys = path[0]
    xe, ye = path[-1]

    eulers = [0,0,0]
    R_min_ = 3.38276381085053

    arr = []

    for i, p in enumerate(path):
        if i < len(path)/2:
            x, y = p
            l2 = (xs-x)*(xs-x) + (ys-y)*(ys-y)
            # print(l2)
            # print((2*R_min_*R_min_ - l2)/(2*R_min_*R_min_))
            eulers[2] = np.arccos((2*R_min_*R_min_ - l2)/(2*R_min_*R_min_))
        else:
            x, y = p
            l2 = (xe-x)*(xe-x) + (ye-y)*(ye-y)
            # print(l2)
            # print((2*R_min_*R_min_ - l2)/(2*R_min_*R_min_))
            eulers[2] = np.arccos((2*R_min_*R_min_ - l2)/(2*R_min_*R_min_))
        arr.append(eulers[2])
        # move(p, quat)
        # time.sleep(0.1)
    print(arr)

    # fig, ax = plt.subplots()
    # ax.plot(xp, yp, 'o', color='black')
    # ax.add_artist(c1)
    # ax.add_artist(c2)
    # ax.set_xlim(-3, 7)
    # ax.set_ylim(-3, 7)
    # plt.show()
