import matplotlib.pyplot as plt
import numpy as np

dist = np.array([63,
                 84,
                 105,
                 147,
                 168,
                 189,
                 210])
pix_count = np.array([29902.94, 16391.52, 10506.04,
                      5164.64, 3904.62, 2951.32, 2534.26])
z = np.polyfit(
    np.array(list(map(lambda x: 1 / np.sqrt(x), pix_count))), dist, 1)
print(z)
plt.plot(np.array(list(map(lambda x: 1 / np.sqrt(x), pix_count))), dist)
plt.show()
