
#include <iostream>
#include <cmath>
#include <vector>
#include <fstream>
#include <iomanip>
#include <algorithm>

using namespace std;

constexpr double PI = 3.141592653589793;

// Coordinates of a target
struct Point {
    double x, y;
};

// Circular obstacle
struct Obstacle {
    double x, y, radius;
};

// Keep angles between -180 and +180 degrees
double normalize(double angle) {
    while (angle > 180) angle -= 360;
    while (angle < -180) angle += 360;
    return angle;
}

// CLASS 1: ROBOT
class robot {
public:
    double x = 0;
    double y = 0;
    double yaw = 0;

    void moveForward(double velocity, double time) {
        double radians = yaw * PI / 180.0;

        x += velocity * cos(radians) * time;
        y += velocity * sin(radians) * time;
    }

    void turn(double angularVelocity, double time) {
        yaw = normalize(yaw + angularVelocity * time);
    }
};

// CLASS 2: CONTROLLER
class controller {
public:
    double targetX = 0;
    double targetY = 0;

    double distance(const robot& r) const {
        double dx = targetX - r.x;
        double dy = targetY - r.y;

        return hypot(dx, dy);
    }

    double desiredYaw(const robot& r) const {
        double dx = targetX - r.x;
        double dy = targetY - r.y;

        return atan2(dy, dx) * 180.0 / PI;
    }
};

// CLASS 3: SENSOR
class sensor {
public:
    vector<Obstacle> obstacles;

    bool collision(double x, double y,
                   double margin = 0.2) const {

        for (const auto& o : obstacles) {
            double d = hypot(x - o.x, y - o.y);

            if (d <= o.radius + margin) {
                return true;
            }
        }
        return false;
    }

    bool clearRay(double x, double y,
                  double angle, double length,
                  double margin = 0.35) const {

        double radians = angle * PI / 180.0;

        for (double d = 0.05; d <= length; d += 0.05) {
            double px = x + d * cos(radians);
            double py = y + d * sin(radians);

            if (collision(px, py, margin)) {
                return false;
            }
        }
        return true;
    }
};

int main() {

    robot r1;
    controller c1;
    sensor s1;

    // Define obstacles: X, Y, radius
    s1.obstacles = {
        {2.5, 2.5, 0.6},
        {5.7, 3.6, 0.6}
    };

    // Define target positions
    vector<Point> waypoints = {
        {5, 5},
        {7, 2},
        {8, 7}
    };

    // Save robot trajectory
    ofstream log("trajectory.csv");

    if (!log) {
        cerr << "Cannot open trajectory.csv\n";
        return 1;
    }

    log << "step,x,y,yaw_deg,target_x,target_y\n";

    // Simulation parameters
    const double dt = 0.1;
    const double speed = 0.6;
    const double maxTurn = 90.0;
    const double tolerance = 0.18;

    int step = 0;

    cout << fixed << setprecision(2);

    // Navigate through each waypoint
    for (size_t w = 0; w < waypoints.size(); ++w) {

        c1.targetX = waypoints[w].x;
        c1.targetY = waypoints[w].y;

        cout << "Waypoint " << w + 1 << ": ("
             << c1.targetX << ", "
             << c1.targetY << ")\n";

        bool reached = false;

        // Navigation control loop
        for (int i = 0; i < 3000; ++i) {

            double dist = c1.distance(r1);

            // Check whether target is reached
            if (dist < tolerance) {
                reached = true;

                cout << "Reached waypoint "
                     << w + 1 << " at ("
                     << r1.x << ", "
                     << r1.y << ")\n";
                break;
            }

            // Calculate direction to target
            double targetHeading = c1.desiredYaw(r1);

            // Obstacle avoidance
            if (!s1.clearRay(
                    r1.x, r1.y,
                    targetHeading, min(dist, 1.3))) {

                bool found = false;

                for (double offset : {
                    35.0, -35.0,
                    65.0, -65.0,
                    95.0, -95.0,
                    135.0, -135.0,
                    180.0
                }) {

                    double candidate =
                        normalize(targetHeading + offset);

                    if (s1.clearRay(
                            r1.x, r1.y,
                            candidate, 1.0)) {

                        targetHeading = candidate;
                        found = true;
                        break;
                    }
                }

                if (!found) {
                    cout << "No safe direction; stopping.\n";
                    return 2;
                }
            }

            // Calculate heading error
            double error =
                normalize(targetHeading - r1.yaw);

            // Calculate angular velocity
            double turnRate =
                clamp(2.0 * error, -maxTurn, maxTurn);

            // Reduce speed during sharp turns
            double velocity =
                (abs(error) > 25.0)
                ? 0.0
                : min(speed, dist);

            // Predict next robot position
            robot candidate = r1;

            candidate.turn(turnRate, dt);
            candidate.moveForward(velocity, dt);

            // Collision prevention
            if (s1.collision(candidate.x, candidate.y)) {
                cout << "Collision prevention stop.\n";
                return 3;
            }

            // Update actual robot position
            r1 = candidate;

            // Record trajectory
            log << step++ << ","
                << r1.x << ","
                << r1.y << ","
                << r1.yaw << ","
                << c1.targetX << ","
                << c1.targetY << "\n";

            if (i % 100 == 0) {
                cout << "  x=" << r1.x
                     << " y=" << r1.y
                     << " yaw=" << r1.yaw
                     << " distance=" << dist << "\n";
            }
        }

        if (!reached) {
            cout << "Timeout: could not reach waypoint "
                 << w + 1 << "\n";
            return 4;
        }
    }

    cout << "Finished! Final position: ("
         << r1.x << ", " << r1.y
         << ") yaw=" << r1.yaw << " degrees\n";

    cout << "Saved trajectory.csv\n";

    return 0;
}
