import matplotlib.pyplot as plt
import numpy
import math

import sys
sys.path.append('../Oncilla')

import OncillaKinematics as OK
import ikm as OK2

alphamin = OK.q_limits[0, 0]
alphamax = OK.q_limits[1, 0]

betamin = OK.q_limits[0, 1]
betamax = OK.q_limits[1, 1]

gammamin = OK.q_limits[0, 2]
gammamax = OK.q_limits[1, 2]

alpha = 0.0*math.pi/180.0
beta = 135.0*math.pi/180.0
gamma = 90.0*math.pi/180.0

f, (ax1, ax2) = plt.subplots(1, 2, sharey=True)
plt.gca().invert_yaxis()

for alpha in numpy.arange(alphamin, alphamax, 0.01):
    for beta in numpy.arange(betamin, betamax, 0.01):
        X = OK.FootPositions_FromServo(alpha, beta, gamma, 1) 
        ax1.plot(X[1,0], X[0,0], 'ko')

ax1.set_xlabel('X')
ax1.set_ylabel('Y')
ax1.set_title('gamma = 0.0')

alpha = 0.0*math.pi/180.0

for gamma in numpy.arange(gammamin, gammamax, 0.0005):
    for beta in numpy.arange(betamin, betamax, 0.01):
        X = OK.FootPositions_FromServo(alpha, beta, gamma, 1) 
        ax2.plot(X[2,0], X[0,0], 'ko')

ax2.set_xlabel('Z')
ax2.set_ylabel('Y')
ax2.set_title('alpha = 0.0')

plt.show()
