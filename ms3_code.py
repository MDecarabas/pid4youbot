
import numpy as np
import modern_robotics as mr

X_errInt = np.zeros(6)

def Feedforward(x, x_d, x_dNext, Kp, Ki, dt, X_errInt):
    X_err = mr.se3ToVec(mr.MatrixLog6(mr.TransInv(x)@x_d))
    Vd = (1/dt)*mr.se3ToVec(mr.MatrixLog6(mr.TransInv(x_d)@x_dNext))
    print('\nVd\n',Vd)
    Ad = mr.Adjoint(mr.TransInv(x)@x_d)
    print('\nAd_tot\n', Ad@Vd)
    X_errInt = X_err*dt + X_errInt
    V = Ad@Vd +Kp@X_err+Ki@X_errInt
    print('\nV\n', V)
    return V, X_err, X_errInt

def jPInv(Tb0, Blist, M0e, config):

    l = 0.47/2  #Length
    w = 0.3/2   #width
    r = 0.0475 #Wheel Radius


    T0e = mr.FKinBody(M0e, Blist, config[3:8])

    F6 =  r/4*np.array([[0, 0, 0, 0], 
                        [0, 0, 0, 0], 
                        [-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w)],
                        [1,  1,  1, 1],
                        [-1, 1, -1, 1], 
                        [0, 0, 0, 0]])
    T0e_Inv = mr.TransInv(T0e)

    Tb0_Inv = mr.TransInv(Tb0)

    jbase = mr.Adjoint(T0e_Inv@Tb0_Inv)@F6

    jarm = mr.JacobianBody(Blist, config[3:8])

    # if config[3] 

    je = np.hstack((jbase, jarm))

    print('\nJe\n', je)

    jpInv = np.linalg.pinv(je)
# ,1e-2

    return jpInv


# phi = 0
# x = 0
# y = 0
# theta1 = 0
# theta2 = 0
# theta3 = 0.2
# theta4 = -1.6
# theta5 = 0


# config = np.array([phi, x, y, theta1, theta2, theta3, theta4, theta5])


# x = np.array([[0.170, 0, 0.985, 0.387],
#               [0, 1, 0, 0],
#               [-0.985, 0, 0.170, 0.570],
#               [0, 0, 0, 1]])

# x_d = np.array([[0, 0, 1, 0.5],
#                 [0, 1, 0, 0],
#                 [-1, 0, 0, 0.5],
#                 [0, 0, 0, 1]])

# x_dNext = np.array([[0, 0, 1, 0.6],
#                     [0, 1, 0, 0],
#                     [-1, 0, 0, 0.3],
#                     [0, 0, 0, 1]])


# Kp_const =0
# Ki_const = 0
# Kp = np.identity(6)*Kp_const
# Ki = np.identity(6)*Ki_const
# dt = 0.01


# Tb0 = np.array([[1, 0, 0, 0.1662],
#                [0, 1, 0, 0],
#                [0, 0, 1, 0.0026],
#                [0, 0, 0, 1]])

# M0e = np.array([[1, 0, 0, 0.033],
#                 [0, 1, 0, 0],
#                 [0, 0, 1, 0.6546],
#                 [0, 0, 0, 1]])

# Blist = np.array([[0, 0, 1, 0, 0.033, 0],
#                   [0, -1, 0, -0.5076, 0, 0], 
#                   [0, -1, 0, -0.3526, 0, 0],
#                   [0, -1, 0, -0.2176, 0, 0], 
#                   [0, 0, 1, 0, 0, 0]]).T
# tta_config = config[3:8]
# jpInv = jPInv(Tb0, Blist, M0e, config)
# V, X_err, X_errInt = Feedforward(x, x_d, x_dNext, Kp, Ki, dt, X_errInt)
# f = jpInv@V
# print('\nf\n', f)