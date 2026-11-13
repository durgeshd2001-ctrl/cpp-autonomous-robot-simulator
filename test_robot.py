
import csv
import math
import subprocess
from pathlib import Path

# Run the C++ simulator
result = subprocess.run(
    ["./robot_sim"],
    capture_output=True,
    text=True
)

assert result.returncode == 0, (
    "Simulator failed:\n" + result.stdout + result.stderr
)

# Read generated trajectory
with open("trajectory.csv", newline="") as file:
    data = list(csv.DictReader(file))

assert len(data) > 0, "Trajectory is empty"

# Test 1: Final target reached
last = data[-1]
x = float(last["x"])
y = float(last["y"])

distance = math.hypot(8 - x, 7 - y)

assert distance < 0.18, "Robot did not reach final waypoint"
print("PASS: Final waypoint reached")

# Test 2: Collision checking
obstacles = [
    (2.5, 2.5, 0.6),
    (5.7, 3.6, 0.6)
]

for row in data:
    rx = float(row["x"])
    ry = float(row["y"])

    for ox, oy, radius in obstacles:
        separation = math.hypot(rx - ox, ry - oy)
        assert separation > radius + 0.2 - 1e-6, (
            "Collision detected in trajectory"
        )

print("PASS: No obstacle collisions")

# Test 3: All waypoints reached
for wx, wy in [(5, 5), (7, 2), (8, 7)]:
    reached = any(
        math.hypot(
            float(row["x"]) - wx,
            float(row["y"]) - wy
        ) < 0.18
        for row in data
    )

    assert reached, f"Waypoint ({wx}, {wy}) not reached"

print("PASS: All waypoints reached")
print("All automated tests passed!")
