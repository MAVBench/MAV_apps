import csv
import numpy as np
import matplotlib.pyplot as plt
file_path = "./dec_data.csv"

velocity = []
deceleration_time = []
distance_travelled = []

with open(file_path, newline='') as csvfile:
    filereader = csv.reader(csvfile)
    next(filereader) # ignore row titles
    for row in filereader:
        velocity.append(float(row[0]))
        deceleration_time.append(float(row[1]))
        distance_travelled.append(float(row[2]))

vel_time, err1, _, _, _ = np.polyfit(velocity, deceleration_time, 2, full=True)
f1 = np.poly1d(vel_time)

vel_dist, err2, _, _, _ = np.polyfit(velocity, distance_travelled, 2, full=True)
f2 = np.poly1d(vel_dist)

print("vel vs time polynomial: ")
print(f1)
print("vel vs time error: " + str(err1[0]))
x = np.linspace(0, 12)
y = list(map(f1, x))
plt.figure(0)
plt.plot(x, y, 'ro')
plt.plot(velocity, deceleration_time, 'bo')
plt.savefig("velocity_vs_deceleration_time.png")

print("vel vs dist polynomial: ")
print(f2)
print("vel vs dist error: " + str(err2[0]))
y = list(map(f2, x))
plt.figure(1)
plt.plot(x, y, 'ro')
plt.plot(velocity, distance_travelled, 'bo')
plt.savefig("velocity_vs_distance_travelled.png")
