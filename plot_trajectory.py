
import csv
import matplotlib.pyplot as plt

x = []
y = []

with open("trajectory.csv", "r") as file:
    reader = csv.DictReader(file)

    for row in reader:
        x.append(float(row["x"]))
        y.append(float(row["y"]))

plt.figure(figsize=(9, 7))

# Robot trajectory
plt.plot(x, y, "b-", linewidth=2, label="Robot Path")

# Starting position
plt.scatter(x[0], y[0], color="green", s=100, label="Start")

# Final position
plt.scatter(x[-1], y[-1], color="red", s=100, label="Finish")

# Target waypoints
waypoints = [(5, 5), (7, 2), (8, 7)]

for i, (wx, wy) in enumerate(waypoints):
    plt.scatter(wx, wy, marker="*", color="orange", s=180)
    plt.annotate(f"WP{i+1}", (wx, wy), xytext=(5, 8),
                 textcoords="offset points")

# Circular obstacles
obstacles = [(2.5, 2.5, 0.6), (5.7, 3.6, 0.6)]

for ox, oy, radius in obstacles:
    circle = plt.Circle(
        (ox, oy),
        radius + 0.2,
        color="gray",
        alpha=0.4
    )
    plt.gca().add_patch(circle)

plt.title("2D Autonomous Robot Navigation Simulator")
plt.xlabel("X Position (m)")
plt.ylabel("Y Position (m)")
plt.axis("equal")
plt.grid(True)
plt.legend()
plt.tight_layout()

plt.savefig("trajectory.png", dpi=200)
plt.show()
