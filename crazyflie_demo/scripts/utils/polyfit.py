#!/usr/bin/env python
import matplotlib.pyplot as plt
import numpy as np

x = [1, 2]
y = [0.3, 0.6]

for x1, y1 in zip(x, y):
    plt.plot(x1, y1, 'ro')

z = np.polyfit(x, y, 1)
f = np.poly1d(z)

print(f)
print('f(20)=', f(20))
for x1 in np.linspace(0, 110, 110):
    plt.plot(x1, f(x1), 'b.')

plt.axis([0, 110, 0, 60])
plt.show()
