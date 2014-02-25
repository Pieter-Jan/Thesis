import sympy 
import numpy
import sympy

A = sympy.Symbol('A')
B = sympy.Symbol('B')
C = sympy.Symbol('C')

Q = sympy.Symbol('Q')
R = sympy.Symbol('R')
S = sympy.Symbol('S')

alpha = sympy.Symbol('alpha')
beta = sympy.Symbol('beta')
gamma = sympy.Symbol('gamma')


DH_BODY = sympy.Matrix(((1.0, 0.0, 0.0, 0.0), 
                        (0.0, 1.0, 0.0, R), 
                        (0.0, 0.0, 1.0, S), 
                        (0.0, 0.0, 0.0, 1.0)))

angle = gamma - sympy.pi/2.0
DH_SERVO = sympy.Matrix(((sympy.cos(angle), 0.0, sympy.sin(angle), 0.0), 
                         (0.0, 1.0, 0.0, 0.0), 
                         (-sympy.sin(angle), 0.0, sympy.cos(angle), 0.0), 
                         (0.0, 0.0, 0.0, 1.0)))

DH_HIP = sympy.Matrix(((sympy.cos(alpha), -sympy.sin(alpha), 0.0, A*sympy.cos(alpha)), 
                       (sympy.sin(alpha), sympy.cos(alpha), 0.0, A*sympy.sin(alpha)), 
                       (0.0, 0.0, 1.0, 0.0), 
                       (0.0, 0.0, 0.0, 1.0)))

DH_KNEE = sympy.Matrix(((sympy.cos(beta+sympy.pi), -sympy.sin(beta+sympy.pi), 0.0, B*sympy.cos(beta+sympy.pi)), 
                        (sympy.sin(beta+sympy.pi), sympy.cos(beta+sympy.pi), 0.0, B*sympy.sin(beta+sympy.pi)),
                        (0.0, 0.0, 1.0, 0.0), 
                        (0.0, 0.0, 0.0, 1.0)))


DH_ANKLE = sympy.Matrix(((sympy.cos(-beta-sympy.pi), -sympy.sin(-beta-sympy.pi), 0.0, sympy.cos(-beta-sympy.pi)*C), 
                         (sympy.sin(-beta-sympy.pi), sympy.cos(-beta-sympy.pi), 0.0, sympy.sin(-beta-sympy.pi)*C), 
                         (0.0, 0.0, 1.0, 0.0), 
                         (0.0, 0.0, 0.0, 1.0)))


tool = sympy.Matrix(((0.0), (0.0), (0.0), (1.0))) # No tool so 0,0,0

tip = sympy.simplify(DH_BODY*DH_SERVO*DH_HIP*DH_KNEE*DH_ANKLE*tool)
tip = tip[0:3,0]

q = sympy.Matrix([[alpha, beta, gamma]])
J = sympy.simplify(tip.jacobian(q))

print(J)

