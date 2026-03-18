# Rapidly-exploring Random Tree Star (RRT*) Implementation for UAV

A robust 3D implementation of the RRT (Rapidly-exploring Random Tree Star) algorithm in Python. Designed for UAV path planning, it efficiently finds optimal trajectories in complex 3D environments by incrementally building a loss-minimizing tree.
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

### Installation
1. Clone the repository:
   ```bash
   git clone [https://github.com/Abhinav8899/Rapidly-exploring-Random-Tree-Star.git](https://github.com/Abhinav8899/Rapidly-exploring-Random-Tree-Star.git)
   cd Rapidly-exploring-Random-Tree-Star
   ```

## 📊 Performance Benchmarking

This repository includes three pre-configured benchmark scenarios to evaluate the algorithm's performance in 3D environments. These scripts measure **execution time**, **node expansion**, and **path optimality**.

### How to Run
Execute the benchmarks directly from your terminal:

```bash
# Scenario 1: Basic environment with low obstacle density
python benchmarksingle1.py

# Scenario 2: Moderate complexity with intersecting obstacles
python benchmarksingle2.py

# Scenario 3: High-complexity stress test with dense 3D obstacle fields
python benchmarksingle3.py
```

## 🤝 Contributing
Feel free to fork this repository and submit pull requests. For major changes, please open an issue first to discuss what you would like to change.

## 📄 License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

