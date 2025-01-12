import numpy as np
path = [(1, 2), (3, 4), (5, 6), (7, 8), (9, 10)]

# random_indices = np.random.choice(len(path), size=2, replace=False)
# random_indices.sort()
random_indices = [1, 4]
# node1, node2 = path[random_indices]

path = path[5:]

print("Selected nodes:", path)