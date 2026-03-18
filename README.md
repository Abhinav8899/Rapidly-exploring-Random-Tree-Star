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
