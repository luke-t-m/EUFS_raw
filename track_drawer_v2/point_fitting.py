import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

x = [0, 2, 3, 0, 5, 9, -1, 6, 7]
y = [0, 0, 1, 1, 4, 4, 3, 2, -5]

t = np.arange(len(x))
ti = np.linspace(0, t.max(), 10 * t.size)

xi = interp1d(t, x, kind='cubic')(ti)
yi = interp1d(t, y, kind='cubic')(ti)

fig, ax = plt.subplots()
ax.plot(xi, yi)
ax.plot(x, y)
ax.margins(0.05)
plt.show()