#!/usr/bin/env python2
'''
simple Kalman filter class

'''

import numpy as np


class KalmanFilter(object):
    '''
    simple Kalman filter set
    all inputs must be numpy arrays
    '''
    def __init__(self, XStart, P, F, G, H, R, Q):

        #State and covariance
        self._XHat = XStart
        self._P = P.reshape(columns, columns)

        #system model
        self._F = F                     # transform matrix
        self._G = G                     # input effect
        self._H = H                     # state change
        self._R = R                     # measurement var
        self._Q = Q                     # process var
        self._I = np.identity(columns)  # Identity matrix

        def _Prediction(self, uPrev):
            XHatMinus = np.dot(self._F, self._XHat) + np.dot(self._G, uPrev)
            PMinus = np.dot(np.dot(self._F, self._P), self._G.T) + self._Q
            return XHatMinus, PMinus


        def _Correction(self, XHatMinus, PMinus, z, y_meas=None):
            if y_meas is None:
                H = self._H
            else:
                H = self._H * y_meas

            Kgain = H.dot(XHatMinus).dot(H.T) + self._R
            Kgain = np.linalg.inv(Kgain)
            Kgain = PMinus.dot(H.T).dot(Kgain)

            XHatPlus = XHatMinus + Kgain.dot((y_meas))


            print "test"

        def Iterate(self, u):
            XHatMinus, PMinus = self._Prediction(u)


def oneD_test():
    import matplotlib.pyplot as plt
    from scipy.linalg import expm
    from numpy.random import multivariate_normal
    from numpy.random import normal

    np.random.seed(100)         # seed for testing
    '''1D robot car'''
    N = 500                     # number of samples
    x0 = np.array([[0,0]])      # state start position
    p0 = np.eye(2)              # Prediction covarience
    dt = .1                     # time step
    tvec = np.arange(0,dt*N, dt)# time vector
    Qtilde = 0                  # process noise intensity
    Rtilde = .05                 #measurement noise intensity

    ''' continous time model'''
    A = np.array([[0, 1],
                  [0, 0]])
    B = np.array([[0],
                  [1]])
    Gamma = np.array([[0],
                      [1]])
    C = np.array([[1, 0]])

    '''discret time model'''
    F = np.array([[1, dt],
                  [0, 1]])
    G = np.array([[.5*dt*dt],
                  [dt]])
    H = np.array([[1, 0]])

    '''use Van Loan to find covariance process noise'''
    M = np.concatenate((-A, Gamma*Qtilde*Gamma.T), axis=1)
    temp = np.concatenate((np.zeros((2,2)), A.T), axis = 1)
    M = np.concatenate((M, temp), axis = 0)
    matrexp_M = expm(M)
    invFQblock = matrexp_M[0:2, 2:4]

    Q = F*invFQblock        #covariance process noise
    R = Rtilde/dt           #measurement noise covariance
    u = 2*np.cos(.75*tvec)     # acceleration input
    '''pure forward prediction'''
    #TODO add covariances??
    pxk = x0
    pxk_hist = np.zeros([2, N])   # predictin x histogram
    for k in range(N):
        pxk1 = np.dot(F, pxk.T) + G*u[k]
        pxk = pxk1.T
        pxk_hist[:, k] = pxk

    '''simulate ground truth data'''
    xk_truehist = np.zeros([2, N])
    ykhist = np.zeros([1, N])
    xk = multivariate_normal(x0[0,:], p0) #initial robot state
    xk = np.expand_dims(xk, axis=0)
    xk = xk.T
    print xk
    for k in range(N):
        '''simulate process noise'''
        wk = multivariate_normal(np.zeros(2), Q)
        wk = np.expand_dims(wk, axis=0)
        xkp1 = np.dot(F, xk) + G*u[k] + wk.T

        '''simulate measurement noise'''
        vk = normal(np.zeros(1), R)
        vk = np.expand_dims(vk, axis=0)
        yk = np.dot(H, xkp1) + vk.T

        '''store and iterate'''
        xk_truehist[0, k] = xkp1[0][0]
        xk_truehist[1, k] = xkp1[1][0]
        ykhist[0, k] = yk[0][0]
        xk = xkp1

    '''show velocity and position predicted vs measured'''
    plt.figure(1)
    plt.subplot(211)
    plt.title('predicted vs measured velocity')
    plt.plot(tvec, pxk_hist[1, :], 'bs', tvec, xk_truehist[1,:], 'r--' )

    plt.subplot(212)
    plt.title('predicted vs measured position')
    plt.plot(tvec, pxk_hist[0, :], 'bs', tvec, xk_truehist[0,:], 'r--' )
    plt.show()

    oneD_kalman = KalmanFilter()



def main():
    oneD_test()



if __name__ == '__main__':
    main()
