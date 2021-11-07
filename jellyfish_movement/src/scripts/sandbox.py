import numpy as np

A = np.uint8(True)
b = A << 1
b = b | A << 2
print(b)

