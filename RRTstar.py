import numpy as np
from typing import List, Tuple, Optional, Union
import random

class Node:
    def __init__(self, position: np.ndarray):
        self.position = position
        self.parent = None
        self.cost = 0.0

class RRTBasicPlanner:
    def __init__(self, 
                 start: np.ndarray,
                 goal: np.ndarray,
                 bounds: np.ndarray,
                 obstacles: List[Union[np.ndarray, Tuple, dict]],
                 max_iterations: int = 20000,
                 step_size: float = 0.5,
                 search_radius: float = 2.0,
                 goal_sample_rate: float = 0.1):
        
        self.start = Node(np.array(start))
        self.goal = Node(np.array(goal))
        self.bounds = np.array(bounds)
        self.obstacles = obstacles
        
        self.nodes = [self.start]
        self.max_iterations = max_iterations
        self.step_size = step_size
        self.search_radius = search_radius
        self.goal_sample_rate = goal_sample_rate

    def check_collision(self, point: np.ndarray) -> bool:
        """Check if point collides with obstacles"""
        for obs in self.obstacles:
            if isinstance(obs, np.ndarray):  # Box obstacle
                x, y, z = point
                ox_min, oy_min, oz_min, ox_max, oy_max, oz_max = obs
                if (ox_min <= x <= ox_max and
                    oy_min <= y <= oy_max and
                    oz_min <= z <= oz_max):
                    return True
            else:  # Spherical obstacle
                if isinstance(obs, dict):
                    center = obs['position']
                    radius = obs['radius']
                else:
                    center, radius = obs
                if np.linalg.norm(point - np.array(center)) <= radius:
                    return True
        return False

    def check_path_collision(self, start_point: np.ndarray, end_point: np.ndarray) -> bool:
        """Basic collision check between two points"""
        if self.check_collision(start_point) or self.check_collision(end_point):
            return True
        return False

    def sample_point(self) -> np.ndarray:
        """Sample a random point in the space"""
        if random.random() < self.goal_sample_rate:
            return self.goal.position
            
        point = np.zeros(3)
        for i in range(3):
            point[i] = random.uniform(self.bounds[i][0], self.bounds[i][1])
        return point

    def find_nearest_node(self, point: np.ndarray) -> Node:
        """Find nearest node in the tree"""
        distances = [np.linalg.norm(node.position - point) for node in self.nodes]
        return self.nodes[np.argmin(distances)]

    def steer(self, from_point: np.ndarray, to_point: np.ndarray) -> np.ndarray:
        """Steer from start point toward end point"""
        vec = to_point - from_point
        dist = np.linalg.norm(vec)
        if dist <= self.step_size:
            return to_point
        return from_point + (vec / dist) * self.step_size

    def find_near_nodes(self, point: np.ndarray) -> List[Node]:
        """Find nodes within search radius"""
        near_nodes = []
        for node in self.nodes:
            if np.linalg.norm(node.position - point) <= self.search_radius:
                near_nodes.append(node)
        return near_nodes

    def calculate_cost(self, from_node: Node, to_point: np.ndarray) -> float:
        """Calculate cost to reach a point"""
        return from_node.cost + np.linalg.norm(from_node.position - to_point)

    def rewire(self, new_node: Node, near_nodes: List[Node]):
        """Rewire the tree"""
        for near_node in near_nodes:
            if near_node == new_node.parent:
                continue
                
            potential_cost = self.calculate_cost(new_node, near_node.position)
            
            if (potential_cost < near_node.cost and
                not self.check_path_collision(new_node.position, near_node.position)):
                near_node.parent = new_node
                near_node.cost = potential_cost

    def plan_path(self) -> Optional[np.ndarray]:
        """Plan the path using basic RRT*"""
        for _ in range(self.max_iterations):
            # Sample random point
            random_point = self.sample_point()
            
            # Find nearest node
            nearest_node = self.find_nearest_node(random_point)
            
            # Steer toward random point
            new_point = self.steer(nearest_node.position, random_point)
            
            # Check collision
            if self.check_path_collision(nearest_node.position, new_point):
                continue
            
            # Create new node
            new_node = Node(new_point)
            
            # Find near nodes
            near_nodes = self.find_near_nodes(new_point)
            
            # Connect to best parent
            min_cost = float('inf')
            best_parent = None
            
            for near_node in near_nodes:
                potential_cost = self.calculate_cost(near_node, new_point)
                if (potential_cost < min_cost and
                    not self.check_path_collision(near_node.position, new_point)):
                    min_cost = potential_cost
                    best_parent = near_node
            
            if best_parent is None:
                continue
            
            # Add new node to tree
            new_node.parent = best_parent
            new_node.cost = min_cost
            self.nodes.append(new_node)
            
            # Rewire tree
            self.rewire(new_node, near_nodes)
            
            # Check if goal is reached
            if np.linalg.norm(new_point - self.goal.position) < self.step_size:
                self.goal.parent = new_node
                self.goal.cost = self.calculate_cost(new_node, self.goal.position)
                
                # Extract and return path
                path = []
                current_node = self.goal
                while current_node is not None:
                    path.append(current_node.position)
                    current_node = current_node.parent
                return np.array(path[::-1])
        
        return None  # No path found