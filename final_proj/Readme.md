# 🚀 Path Planning Algorithms Comparison

This repository contains a comparative analysis of various path planning algorithms, including **RRT**, **RRT-Connect**, and **Improved RRT-Connect**. The study evaluates these algorithms based on efficiency, execution time, and path optimization.

## 📽 Algorithm Demonstrations

Below are the MP4 video demonstrations of each algorithm's execution. Click on the videos to watch them in action.

<table>
  <tr>
    <th>RRT Algorithm</th>
    <th>RRT-Connect Algorithm</th>
    <th>Improved RRT-Connect Algorithm</th>
  </tr>
  <tr>
    <td><video width="320" height="240" controls><source src="videos/RRT.mp4" type="video/mp4"></video></td>
    <td><video width="320" height="240" controls><source src="videos/RRT-Connect.mp4" type="video/mp4"></video></td>
    <td><video width="320" height="240" controls><source src="videos/Improved-RRT-Connect.mp4" type="video/mp4"></video></td>
  </tr>
</table>

---

## 📌 Introduction

Path planning algorithms play a crucial role in robotics for autonomous navigation. This repository explores different algorithms:

- **RRT (Rapidly Exploring Random Trees):** A probabilistic algorithm that efficiently searches high-dimensional spaces but may not produce optimal paths.
- **RRT-Connect:** An improved version of RRT that expands trees bidirectionally, reducing search time.
- **Improved RRT-Connect:** A refined version with goal-biased sampling and adaptive step size to enhance convergence speed and path smoothness.

---

## 📊 Algorithm Comparison

| Algorithm                 | Execution Time ⏱ | Path Optimality 📏 | Efficiency 🚀 |
|---------------------------|-----------------|-------------------|--------------|
| **RRT**                   | ⏳ Slow         | ❌ Less Optimal  | 🔴 Low       |
| **RRT-Connect**           | ⚡ Faster       | ✅ More Optimal  | 🟠 Medium    |
| **Improved RRT-Connect**  | 🚀 Fastest      | ✅✅ Highly Optimal | 🟢 High       |

---

## 📜 How to Run the Simulations

### Prerequisites
- Python 3.x
- OpenCV
- Matplotlib
- NumPy

### Running the Algorithms
```sh
python run_rrt.py
python run_rrt_connect.py
python run_improved_rrt_connect.py
