import matplotlib.pyplot as plt
import numpy
from matplotlib.patches import Polygon

fig, ax = plt.subplots()

P1 = numpy.array([0.0, 0.0])
P2 = numpy.array([4.0, 0.0])
P3 = numpy.array([2.0, 4.0])
ax.add_patch(Polygon([P1, P2, P3], closed=True))

margin = 0.5

# Unit vectors 
u21 = (P2 - P1)/numpy.linalg.norm(P2 - P1)
u31 = (P3 - P1)/numpy.linalg.norm(P3 - P1)
u32 = (P3 - P2)/numpy.linalg.norm(P3 - P2)

# Points on new lines
P1_acc = P1 + u31*margin + u21*margin
P2_acc = P2 - u21*margin + u32*margin
P3_acc = P3 - u32*margin - u31*margin

# ax.plot(P1_acc[0], P1_acc[1], 'ro')
# ax.plot(P2_acc[0], P2_acc[1], 'ro')
# ax.plot(P3_acc[0], P3_acc[1], 'ro')

ax.add_patch(Polygon([P1_acc, P2_acc, P3_acc], color='red', closed=True))

plt.axis('equal')
plt.axis([0.0, 5.0, 0.0, 5.0])
plt.show()


