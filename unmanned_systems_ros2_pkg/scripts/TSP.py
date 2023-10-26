import itertools as ittyboys
import math as m
import matplotlib.pyplot as plt
import pandas as pd

start = (0, 0)
goals = [(2, 2), (5, 3), (3, 4), (6, 4)]

def calculate_distance(point1, point2):
    return m.dist(point1, point2)

wp = list(range(len(goals)))
permutations = ittyboys.permutations(wp)

best_path = None
min_distance = float('inf')

distances = {}
for perms in permutations:
    total_distance = calculate_distance(start, goals[perms[0]])
    for i in range(len(perms) - 1):
        total_distance += calculate_distance(goals[perms[i]], goals[perms[i + 1]])
    distances[perms] = total_distance

for perm, total_distance in distances.items():
    if total_distance < min_distance:
        min_distance = total_distance
        best_path = perm

best_path_xy = [start] + [goals[i] for i in best_path]

x_values, y_values = zip(*best_path_xy)
plt.plot(x_values, y_values, marker='o', label='Optimal Path')
plt.annotate('Start', start)
for i, (x, y) in enumerate(best_path_xy[1:], start=1):
    plt.annotate(f'Goal {i}', (x, y))

plt.title(f"TSP (Total Distance: {min_distance:.2f})")
plt.grid()

df = pd.DataFrame(list(distances.items()), columns=['Path', 'Distance'])
fig, ax = plt.subplots()
ax.axis('off')
table = ax.table(cellText=df.values, colLabels=df.columns, loc='center', cellLoc='center')

plt.show()
