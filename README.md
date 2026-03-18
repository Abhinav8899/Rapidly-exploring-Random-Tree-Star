# Rapidly-exploring Random Tree Star (RRT*) Implementation

A robust Python implementation of the **RRT*** algorithm designed for 3D path planning. This implementation efficiently finds optimal paths in environments with complex obstacles by incrementally building a loss-minimizing tree and "rewiring" nodes to shorten the total path length.

## 🛠 Key Features
* **3D Path Planning:** Full support for three-dimensional coordinate systems.
* **Dual Collision Detection:** Handles both **Spherical** and **Box** (AABB) obstacles.
* **Tree Optimization:** Implements the RRT* "rewire" step to ensure path cost decreases over iterations.
* **Modular Design:** Easy to integrate into robotics simulations or autonomous navigation stacks.

## 🚀 Getting Started

### Prerequisites
* Python 3.x
* NumPy

```bash
pip install numpy
Installation
Clone the repository:

Bash
git clone [https://github.com/Abhinav8899/Rapidly-exploring-Random-Tree-Star.git](https://github.com/Abhinav8899/Rapidly-exploring-Random-Tree-Star.git)
cd Rapidly-exploring-Random-Tree-Star
💡 Usage Example
You can initialize the RRTBasicPlanner with custom boundaries and obstacles as shown below:

Python
import numpy as np
from RRTstar import RRTBasicPlanner

# 1. Define space boundaries [[min_x, max_x], [min_y, max_y], [min_z, max_z]]
bounds = np.array([[0, 10], [0, 10], [0, 10]])

# 2. Define obstacles (Box arrays or Sphere dictionaries)
obstacles = [
    np.array([2, 2, 2, 4, 4, 4]),             # Box: [x1, y1, z1, x2, y2, z2]
    ({'position': [6, 6, 6], 'radius': 1.5})  # Sphere dictionary
]

# 3. Initialize Planner
planner = RRTBasicPlanner(
    start=np.array([0, 0, 0]),
    goal=np.array([9, 9, 9]),
    bounds=bounds,
    obstacles=obstacles,
    max_iterations=10000,
    step_size=0.5
)

# 4. Plan the path
path = planner.plan_path()

if path is not None:
    print(f"Success! Path found with {len(path)} nodes.")
    print(path)
else:
    print("No path found within the iteration limit.")
📊 Benchmarking
This repository includes specialized scripts to test performance across different obstacle densities. Run these to evaluate execution time and path optimality:

Bash
python benchmarksingle1.py
python benchmarksingle2.py
python benchmarksingle3.py
