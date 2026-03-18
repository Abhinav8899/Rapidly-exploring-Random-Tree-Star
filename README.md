# Rapidly-exploring Random Tree Star (RRT*) Implementation

A robust Python implementation of the **RRT*** algorithm designed for 3D path planning. This implementation focuses on finding optimal paths in environments with complex obstacles by incrementally building a loss-minimizing tree.


## 🛠 Key Features
* **3D Path Planning:** Operates in a three-dimensional coordinate system.
* **Collision Detection:** Supports both **Spherical** and **Box** (Axis-Aligned Bounding Box) obstacles.
* **Optimization:** Implements the "Rewire" step unique to RRT* to ensure path cost decreases over time.
* **Flexible Sampling:** Configurable goal-sampling rate to bias the search toward the destination.

## 🚀 Getting Started

### Prerequisites
* Python 3.x
* NumPy

```bash
pip install numpy

### Installation
1. Clone the repository:
   ```bash
git clone [https://github.com/Abhinav8899/Rapidly-exploring-Random-Tree-Star.git](https://github.com/Abhinav8899/Rapidly-exploring-Random-Tree-Star.git)
cd Rapidly-exploring-Random-Tree-Star

## 💡 Usage

The RRTBasicPlanner class is designed to be modular. Below is a snippet to initialize a planner with obstacles:
import numpy as np
from RRTstar import RRTBasicPlanner

# Define space boundaries [[min_x, max_x], [min_y, max_y], [min_z, max_z]]
bounds = np.array([[0, 10], [0, 10], [0, 10]])

# Define obstacles (List of arrays for boxes or tuples for spheres)
obstacles = [
    np.array([2, 2, 2, 4, 4, 4]), # Box obstacle
    ({'position': [6, 6, 6], 'radius': 1.5}) # Sphere obstacle
]

planner = RRTBasicPlanner(
    start=np.array([0, 0, 0]),
    goal=np.array([9, 9, 9]),
    bounds=bounds,
    obstacles=obstacles
)

path = planner.plan_path()

if path is not None:
    print(f"Path found with {len(path)} nodes!")

