#!/usr/bin/env python
import matplotlib.pyplot as plt
import numpy as np
import os

f = 0.66666667
amplitude = 0.25
zheight = 0
order = 7

t = np.linspace(0, 1 / f, 100)
z = zheight + amplitude * np.sin(2 * np.pi * f * t)

plt.plot(t, z, 'r.')

pfit = np.polyfit(t, z, order)
fit = np.poly1d(pfit)

print(fit)
print('f(start)=', fit(0))
print('f(end)=', fit(1 / f))
for x1 in t:
    plt.plot(x1, fit(x1), 'b+')

plt.axis([0, 2, -2, 2])
plt.show()

output_file = os.getenv("HOME") + "/route.csv"

matrix = np.empty([0, order + 1])

row = np.array([])
row = np.append(row, 1 / f)
polynomial = pfit[::-1]

row = np.append(row, polynomial)
row = row.reshape(1, row.shape[0])
print('writing to file - ', row)
np.savetxt(output_file, row, delimiter=",", header="t,x0,x1,x2,x3,x4,x5,x6,x7")
