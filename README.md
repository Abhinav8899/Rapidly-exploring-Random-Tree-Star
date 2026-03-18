# Rapidly-exploring Random Tree Star (RRT*) Implementation

A robust Python implementation of the **RRT*** algorithm designed for 3D path planning. This implementation efficiently finds optimal paths in environments with complex obstacles by incrementally building a loss-minimizing tree.

## 🚀 Getting Started

### Prerequisites
* Python 3.x
* NumPy

## 🧠 Algorithm Overview
RRT* (Rapidly-exploring Random Tree Star) is an incremental sampling-based motion planning algorithm. It improves upon the basic RRT by ensuring **asymptotic optimality**.

### Key Steps in this Implementation:
1. **Sampling**: A random configuration is sampled from the 3D search space.
2. **Nearest Node**: The algorithm finds the existing node in the tree closest to the sample.
3. **Steering**: A new node is created by moving from the nearest node toward the sample by a fixed `step_size`.
4. **Choose Parent**: Instead of just connecting to the nearest node, it searches a `search_radius` to find the neighbor that offers the lowest path cost from the start.
5. **Rewiring**: This implementation checks if the new node can provide a shorter path to any of its existing neighbors. If so, those neighbors are "rewired" to the new node.

## 🤝 Contributing
Feel free to fork this repository and submit pull requests. For major changes, please open an issue first to discuss what you would like to change.

## 💡 Usage Example

You can initialize the `RRTBasicPlanner` with custom boundaries and obstacles. The planner supports both 3D boxes and spherical obstacles.

```python
import numpy as np
from RRTstar import RRTBasicPlanner

# 1. Define search space boundaries [[min_x, max_x], [min_y, max_y], [min_z, max_z]]
bounds = np.array([[0, 10], [0, 10], [0, 10]])

# 2. Define obstacles 
# Box: np.array([x_min, y_min, z_min, x_max, y_max, z_max])
# Sphere: {'position': [x, y, z], 'radius': r}
obstacles = [
    np.array([2, 2, 2, 4, 4, 4]),             # Central Box Obstacle
    ({'position': [6, 6, 6], 'radius': 1.5})  # Floating Sphere Obstacle
]

# 3. Initialize the Planner
planner = RRTBasicPlanner(
    start=np.array([0, 0, 0]),
    goal=np.array([9, 9, 9]),
    bounds=bounds,
    obstacles=obstacles,
    max_iterations=10000,
    step_size=0.5,
    search_radius=2.0
)

# 4. Generate the optimal path
path = planner.plan_path()

if path is not None:
    print(f"✅ Success! Path found with {len(path)} nodes.")
    print("Path coordinates:\n", path)
else:
    print("❌ No path found within the iteration limit.")
