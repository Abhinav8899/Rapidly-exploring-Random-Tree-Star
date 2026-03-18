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
