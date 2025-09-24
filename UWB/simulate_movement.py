import matplotlib.pyplot as plt
import numpy as np
from base_station import BaseStation
from biliterate import trilaterate_two_anchors

def simulate_robot():
    #Initialize base stations:

    pathfinder1 = BaseStation("A","pathfinder1","00",0,0)
    pathfinder2 = BaseStation("B","pathfinder2","01",0,5)

    # Ground truth robot path (for simulation only)
    true_positions = [(0, 0)]
    for t in range(1, 100):
        x = t * 0.3
        y = 6 * np.sin(t * 0.3)
        true_positions.append((x, y))

    # Simulated encoder-based and encoder + triliteration dead reckoning
    tri_encoder_estimate = [(0, 0)]
    encoder_estimate = [(0,0)]
    position = np.array([0.0, 0.0])

    # Simulated encoder drift
    drift = np.array([0.001, -0.0005])
    

    for t in range(1, len(true_positions)):
        # Simulate encoder motion (with slight noise)
        dx = true_positions[t][0] - true_positions[t-1][0]
        dy = true_positions[t][1] - true_positions[t-1][1]
        movement = np.array([dx, dy])

        bias = drift * t
        noise = np.random.normal(0, 0.002, size=2)  # small random noise
        error = bias + noise

        # Apply to movement
        estimated_movement = movement + error
        position += estimated_movement
        encoder_estimate.append(tuple(position))

        # Simulate getting distances from anchors (with noise)
        if t % 2 == 0:  # every 2 time steps
            true_x, true_y = true_positions[t]
            d1 = np.hypot(true_x - pathfinder1.x, true_y - pathfinder1.y) + np.random.normal(0, 0.05)
            d2 = np.hypot(true_x - pathfinder2.x, true_y - pathfinder2.y) + np.random.normal(0, 0.05)
            pathfinder1.update_dist(d1)
            pathfinder2.update_dist(d2)
            sols = trilaterate_two_anchors(pathfinder1, pathfinder2)

            if sols is not None:
                # Pick the trilateration solution closer to current estimate
                dist1 = np.linalg.norm(np.array(sols[0]) - position)
                dist2 = np.linalg.norm(np.array(sols[1]) - position)
                corrected = sols[0] if dist1 < dist2 else sols[1]
                position = np.array(corrected)

        tri_encoder_estimate.append(tuple(position))

    return true_positions, encoder_estimate, tri_encoder_estimate

# Run and plot

true_path, encoder_estimated_path, tri_encoder_estimated_path = simulate_robot()
true_x, true_y = zip(*true_path)
est_x, est_y = zip(*encoder_estimated_path)
tri_est_x, tri_est_y = zip(*tri_encoder_estimated_path)

plt.plot(true_x, true_y, 'g-', label='True Path')
plt.plot(est_x, est_y, 'b--', label='Encoder Estimated Path')
plt.plot(tri_est_x, tri_est_y, 'r--', label='Encoder + Trilieration Estimated Path')
plt.scatter([0, 5], [0, 0], c='r', label='Base Stations')
plt.legend()
plt.xlabel("X position")
plt.ylabel("Y position")
plt.title("Pose Estimate Simulation")
plt.axis('equal')
plt.grid(True)
plt.show()
