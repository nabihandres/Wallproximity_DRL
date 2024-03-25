# RAL 
## Modifications to Reward Function and Wall Proximity Subfunctions

### Overview
This README outlines the test environments: blockpath and curvepath. These modifications are centered around the Reward function and the introduction of new subfunctions to better handle robot navigation with respect to wall proximity.

## Curvepath Environment
### No Obstacles
<table>
  <tr>
    <td><img src="https://github.com/nabihandres/sarl_UR2024/blob/main/CrowdNav-curvepath/videos/no.gif" alt="Alt Text 1" width="300"/></td>
    <td><img src="https://github.com/nabihandres/sarl_UR2024/blob/main/CrowdNav-curvepath/videos/no UR.gif" alt="Alt Text 1" width="300"/></td>
    <td><img src="https://github.com/nabihandres/sarl_UR2024/blob/main/CrowdNav-curvepath/videos/no ICCAS.gif" alt="Alt Text 1" width="300"/></td>
  </tr>
  <tr>
    <td>output_alisher_curvepath_5_6</td>
    <td>output_UR</td>
    <td>output_alisher_ICCAS2023</td>
  </tr>
</table>

### Static Obstacles
python test.py --policy sarl --model_dir data/output --phase test --visualize --test_case 1
<table>
  <tr>
    <td><img src="https://github.com/nabihandres/sarl_UR2024/blob/main/CrowdNav-curvepath/videos/static.gif" alt="Alt Text 3" width="300"/></td>
    <td><img src="https://github.com/nabihandres/sarl_UR2024/blob/main/CrowdNav-curvepath/videos/static UR.gif" alt="Alt Text 4" width="300"/></td>
    <td><img src="https://github.com/nabihandres/sarl_UR2024/blob/main/CrowdNav-curvepath/videos/static ICCAS.gif" alt="Alt Text 5" width="300"/></td>
  </tr>
  <tr>
    <td>output_alisher_curvepath_5_6</td>
    <td>output_UR</td>
    <td>output_alisher_ICCAS2023</td>
  </tr>
</table>

### Dynamic Obstacles
python test.py --policy sarl --model_dir data/output --phase test --visualize --test_case 18
<table>
  <tr>
    <td><img src="https://github.com/nabihandres/sarl_UR2024/blob/main/CrowdNav-curvepath/videos/dynamic.gif" alt="Alt Text 3" width="300"/></td>
    <td><img src="https://github.com/nabihandres/sarl_UR2024/blob/main/CrowdNav-curvepath/videos/dynamic UR.gif" alt="Alt Text 4" width="300"/></td>
    <td><img src="https://github.com/nabihandres/sarl_UR2024/blob/main/CrowdNav-curvepath/videos/dynamic ICCAS.gif" alt="Alt Text 5" width="300"/></td>
  </tr>
  <tr>
    <td>output_alisher_curvepath_5_6</td>
    <td>output_UR</td>
    <td>output_alisher_ICCAS2023</td>
  </tr>
</table>

### Mixed Obstacles
python test.py --policy sarl --model_dir data/output --phase test --visualize --test_case 3
<table>
  <tr>
    <td><img src="https://github.com/nabihandres/sarl_UR2024/blob/main/CrowdNav-curvepath/videos/mixed.gif" alt="Alt Text 3" width="300"/></td>
    <td><img src="https://github.com/nabihandres/sarl_UR2024/blob/main/CrowdNav-curvepath/videos/mixed UR.gif" alt="Alt Text 4" width="300"/></td>
    <td><img src="https://github.com/nabihandres/sarl_UR2024/blob/main/CrowdNav-curvepath/videos/mixed ICCAS.gif" alt="Alt Text 5" width="300"/></td>
  </tr>
  <tr>
    <td>output_alisher_curvepath_5_6</td>
    <td>output_UR</td>
    <td>output_alisher_ICCAS2023</td>
  </tr>
</table>

## Crosspath Environment
### No Obstacles
<table>
  <tr>
    <td><img src="https://github.com/nabihandres/sarl_UR2024/blob/main/CrowdNav-crosspath/videos/no.gif" alt="Alt Text 1" width="300"/></td>
    <td><img src="https://github.com/nabihandres/sarl_UR2024/blob/main/CrowdNav-crosspath/videos/no UR.gif" alt="Alt Text 1" width="300"/></td>
    <td><img src="https://github.com/nabihandres/sarl_UR2024/blob/main/CrowdNav-crosspath/videos/no ICCAS.gif" alt="Alt Text 1" width="300"/></td>
  </tr>
  <tr>
    <td>output_alisher_crosspath</td>
    <td>output_UR</td>
    <td>output_alisher_ICCAS2023</td>
  </tr>
</table>

### Circle Crossing
python test.py --policy sarl --model_dir data/output --phase test --visualize --test_case 12
<table>
  <tr>
    <td><img src="https://github.com/nabihandres/sarl_UR2024/blob/main/CrowdNav-crosspath/videos/circle_crossing.gif" alt="Alt Text 1" width="300"/></td>
    <td><img src="https://github.com/nabihandres/sarl_UR2024/blob/main/CrowdNav-crosspath/videos/circle_crossing UR.gif" alt="Alt Text 1" width="300"/></td>
    <td><img src="https://github.com/nabihandres/sarl_UR2024/blob/main/CrowdNav-crosspath/videos/circle_crossing ICCAS.gif" alt="Alt Text 1" width="300"/></td>
  </tr>
  <tr>
    <td>output_alisher_crosspath</td>
    <td>output_UR</td>
    <td>output_alisher_ICCAS2023</td>
  </tr>
</table>

### Static Obstacles
python test.py --policy sarl --model_dir data/output --phase test --visualize --test_case 7
<table>
  <tr>
    <td><img src="https://github.com/nabihandres/sarl_UR2024/blob/main/CrowdNav-crosspath/videos/static.gif" alt="Alt Text 1" width="300"/></td>
    <td><img src="https://github.com/nabihandres/sarl_UR2024/blob/main/CrowdNav-crosspath/videos/static UR.gif" alt="Alt Text 1" width="300"/></td>
    <td><img src="https://github.com/nabihandres/sarl_UR2024/blob/main/CrowdNav-crosspath/videos/static ICCAS.gif" alt="Alt Text 1" width="300"/></td>
  </tr>
  <tr>
    <td>output_alisher_crosspath</td>
    <td>output_UR</td>
    <td>output_alisher_ICCAS2023</td>
  </tr>
</table>

### Square Crossing
python test.py --policy sarl --model_dir data/output --phase test --visualize --test_case 3
<table>
  <tr>
    <td><img src="https://github.com/nabihandres/sarl_UR2024/blob/main/CrowdNav-crosspath/videos/square_crossing.gif" alt="Alt Text 1" width="300"/></td>
    <td><img src="https://github.com/nabihandres/sarl_UR2024/blob/main/CrowdNav-crosspath/videos/square_crossing UR.gif" alt="Alt Text 1" width="300"/></td>
    <td><img src="https://github.com/nabihandres/sarl_UR2024/blob/main/CrowdNav-crosspath/videos/square_crossing ICCAS.gif" alt="Alt Text 1" width="300"/></td>
  </tr>
  <tr>
    <td>output_alisher_crosspath</td>
    <td>output_UR</td>
    <td>output_alisher_ICCAS2023</td>
  </tr>
</table>

### Circle Static
python test.py --policy sarl --model_dir data/output --phase test --visualize --test_case 10
<table>
  <tr>
    <td><img src="https://github.com/nabihandres/sarl_UR2024/blob/main/CrowdNav-crosspath/videos/circle_static.gif" alt="Alt Text 1" width="300"/></td>
    <td><img src="https://github.com/nabihandres/sarl_UR2024/blob/main/CrowdNav-crosspath/videos/circle_static UR.gif" alt="Alt Text 1" width="300"/></td>
    <td><img src="https://github.com/nabihandres/sarl_UR2024/blob/main/CrowdNav-crosspath/videos/circle_static ICCAS.gif" alt="Alt Text 1" width="300"/></td>
  </tr>
  <tr>
    <td>output_alisher_crosspath</td>
    <td>output_UR</td>
    <td>output_alisher_ICCAS2023</td>
  </tr>
</table>

### Square Static
python test.py --policy sarl --model_dir data/output --phase test --visualize --test_case 11
<table>
  <tr>
    <td><img src="https://github.com/nabihandres/sarl_UR2024/blob/main/CrowdNav-crosspath/videos/square_static.gif" alt="Alt Text 1" width="300"/></td>
    <td><img src="https://github.com/nabihandres/sarl_UR2024/blob/main/CrowdNav-crosspath/videos/square_static UR.gif" alt="Alt Text 1" width="300"/></td>
    <td><img src="https://github.com/nabihandres/sarl_UR2024/blob/main/CrowdNav-crosspath/videos/square_static ICCAS.gif" alt="Alt Text 1" width="300"/></td>
  </tr>
  <tr>
    <td>output_alisher_crosspath</td>
    <td>output_UR</td>
    <td>output_alisher_ICCAS2023</td>
  </tr>
</table>

## Reward Function of different outputs
- **output_UR**:

$$
R(J_t, a_t) = 
\begin{cases} 
10 & \text{if Reach the goal} \\
-10 & \text{if Time out} \\
-2 & \text{if Collision} \\
0.5(d_{i} - d_{\text{und}}) & \text{if Uncomfortable} \\
0.01(d_{g}(t - 1) - d_{g}(t)) & \text{otherwise}
\end{cases}
$$

- **output_ICCAS**:

$$
R(J, a) = R_g + R_c + R_{for} + R_d + 2R_s
$$

- **output_alisher**:

$$
R(J, a) = R_g + R_c + R_{for} + R_{km} + R_d + R_s + R_{wall}
$$

## Reward Function Changes
- **Forward Subfunction Modification**: The forward subfunction of the Reward function has been updated to more effectively evaluate the robot's performance based on the distance traveled and the proximity to the goal. The revised function is as follows:
  
  ```python
  R_for += 3*(1 / (1+np.exp(3*end_dg-5.5)))**0.1
  
## Wall Proximity Subfunctions
The Wall Proximity Subfunctions aim to ensure that the robot maintains a safe distance from walls. The robot is penalized for being too close or too far from a wall and rewarded for maintaining an optimal distance.
Environment (crosspath and blockpath) has blocks and each block can be divided to walls or line segments.
The subfunctions include:

#### `compute_min_distance_to_block(self, robot_position, wall_areas)`
Computes the minimum distance from the robot to the nearest block (wall area).
In this function, we iterate through all blocks in the environment and call *compute_distance_to_wall(robot_position, wall_area)* function for each wall area.
- `robot_position`: Current position of the robot.
- `wall_areas`: List of wall area coordinates.
  ```python
  def compute_min_distance_to_block(self, robot_position, wall_areas):
      min_distance = float('inf')
      max_distance = 0

      for wall_area in wall_areas:
          distance_to_wall, point_on_line = self.compute_distance_to_wall(robot_position, wall_area)
          if distance_to_wall < min_distance:
              min_distance = distance_to_wall
              closest_point = point_on_line

      return min_distance, closest_point

#### `compute_distance_to_wall(self, robot_position, wall_area)`
Calculates the distance from the robot to a specified wall area.
In this function, we iterate though all walls or line segments that the block has and call *point_to_line_segment_distance(x, y, x1, y1, x2, y2)* function.
- `robot_position`: Current position of the robot.
- `wall_area`: Coordinates of a specific wall area.
  ```python
  def compute_distance_to_wall(self, robot_position, wall_area):
	    x, y = robot_position
	    distances = {}
	    distances_list = []

	    for i in range(len(wall_area)):
	        x1, y1 = wall_area[i]
	        x2, y2 = wall_area[(i + 1) % len(wall_area)]  
	        distance, point = self.point_to_line_segment_distance(x, y, x1, y1, x2, y2)
	        distances[distance] = point
	        distances_list.append(distance)

	    min_dist = min(distances_list)
	    min_point = distances[min_dist]

	    return min_dist, min_point

#### `point_to_line_segment_distance(self, x, y, x1, y1, x2, y2)`
Calculates the shortest distance from a point to a line segment.
In this function, we are given coordinates of the wall (line segment) and we calculate distance from a robot to that line segment.
- `(x, y)`: Point coordinates of the robot.
- `(x1, y1)`, `(x2, y2)`: Endpoints of the line segment.
  ```python
  def point_to_line_segment_distance(self, x, y, x1, y1, x2, y2):
      length_squared = (x2 - x1)**2 + (y2 - y1)**2
      if length_squared == 0:
          return math.sqrt((x - x1)**2 + (y - y1)**2)

      t = max(0, min(1, ((x - x1) * (x2 - x1) + (y - y1) * (y2 - y1)) / length_squared))
      px = x1 + t * (x2 - x1)
      py = y1 + t * (y2 - y1)
      distance = math.sqrt((x - px)**2 + (y - py)**2)

      return distance, [px, py]

#### `farthest_point_and_distance(self, robot_position, wall_areas)`
Finds the farthest point and distance from the robot to the wall areas.
- `robot_position`: Current position of the robot.
- `wall_areas`: List of wall area coordinates.
  ```python
  def farthest_point_and_distance(self, robot_position, wall_areas):
      farthest_point = None
      max_distance = float('-inf')

      for area in wall_areas:
          for vertex in area:
              dist = self.distance(robot_position, vertex)
              if dist > max_distance:
                  max_distance = dist
                  farthest_point = vertex

      return farthest_point, max_distance

#### `distance(self, point1, point2)`
Calculates the Euclidean distance between two points.
- `point1`, `point2`: Coordinates of the two points.
  ```python
  def distance(self, point1, point2):
      return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)


### Reward/Penalty Mechanism
The robot's behavior is influenced by the following reward/penalty mechanism:

#### `R_wall_min_dist`
- If the robot is too close to a wall (`< 0.01` m), it receives a penalty of `-0.5`.
- Otherwise, it gets a small reward of `0.005`.
  ```python
  min_distance, closest_point = self.compute_min_distance_to_block(self.robot.get_position(), self.blocks)
  min_distance -= self.robot.radius
  R_wall_min_dist = 0
  min_wall_bool = False
  if min_distance < 0.01:
      R_wall_min_dist = -0.5
      min_wall_bool = True
  else:
      R_wall_min_dist = 0.005
  
#### `R_wall_max_dist`
- If the robot is too far from a wall (`> 20` m), it receives a penalty of `-0.5`.
  ```python
  farthest_point, max_distance = self.farthest_point_and_distance(self.robot.get_position(), self.blocks)
  R_wall_max_dist = 0
  if max_distance > 20:	
      R_wall_max_dist = -0.5

## Example of How Wall Proximity Subfunctions Work
![robot_movement](robot_movement.gif)
- Code for plotting is provided in plotting.py
  
## Enhanced Logging Features

### Detailed Reward/Penalty Logging
- **Individual Subfunction Rewards/Penalties**: Each subfunction's contribution to the total reward or penalty is now logged separately. This includes the Wall Proximity Subfunctions and the updated Reward function.
- **Total Reward**: The cumulative reward for each scenario is logged, providing an overview of the robot's overall performance.

### Navigation Metrics
- **Distance Traveled**: The total distance traveled by the robot in each scenario is logged, offering insight into its movement efficiency.
- **Distance Left to Goal**: This metric logs the remaining distance to the goal at the end of each scenario, helping to evaluate how close the robot was to completing its objective.
- **Number of Stops**: The Explorer now logs the number of times the robot stops during a scenario, which can be indicative of its decision-making process and efficiency.
- **Last Position Coordinates**: The final coordinates of the robot at the end of each scenario are recorded. This data is crucial for understanding the robot's final location relative to its intended goal.

### Detailed description
- **Track rewards and penalties for each episode**: To track rewards and penalties for each subfunction in each episode, rewards_dict was introduced to track various components
    ```python
    rewards_dict = {
    "Total Reward": 0,
    "R_dan": 0,
    "2*R_stop": 0,
    "R_forward": 0,
    "R_goal": 0,
    "R_col": 0,
    "R_km": 0,
    "R_col_wall": 0,
    "R_wall_min_dist": 0,
    "R_stop_t": 0,
    "R_for": 0
    }
  
- During each episode (while not done), the rewards_dict is updated based on the returned reward_values from the step function:
  ```python
  rewards_dict = {key: round(rewards_dict[key] + reward_values[key], 3) for key in rewards_dict}
  
#### **Calculation of Metrics:**
- Distance Traveled by Robot is calculated based on the action taken:
  ```python
  if isinstance(action, ActionXY):
    length += 0.25 * np.linalg.norm([action.vx, action.vy])
  else:
    length += 0.25 * action.v
- Detection of Robot Stops:
  A boolean indicating whether the robot is stopped (is_stopped) is returned from the step function and used to count stops:
  ```python
  if is_stopped:
    stops += 1
- Robot Position is retrieved within the step function and utilized in the Explorer function to display the last coordinates:
  ```python
  position = self.robot.get_position()  # Retrieval of robot's position
  # (returned and used in Explorer function for display)
- Distance Remaining to Goal is calculated within the step function using Euclidean distance calculations:
  ```python
  # step function:
  end_position = np.array(self.robot.compute_position(action, self.time_step))
  start_dg = norm(self.robot.get_position() - np.array(self.robot.get_goal_position()))
  end_dg = norm(end_position - np.array(self.robot.get_goal_position()))
  left_path = end_dg - self.robot.radius
  # (returned to Explorer function for further utilization)

### Implementation
The logging enhancements are integrated into the Explorer file. During training, these metrics are recorded for each scenario.

## Modified ORCA Function

### Circular Obstacle Approximation
To simulate circular obstacles, each circle is approximated by a series of connected line segments. This method allows the existing ORCA algorithm to handle circular obstacles as if they were a series of straight-line obstacles. Here is the implementation detail:

```python
  circle_center = (-8, -8)
  circle_radius1 = 10.0
  circle_radius2 = 15.0

  num_segments = 30
  for circle_radius in [circle_radius1, circle_radius2]:
      for i in range(num_segments):
          angle = 2 * math.pi * i / num_segments
          next_angle = 2 * math.pi * (i + 1) / num_segments

          x1 = circle_center[0] + circle_radius * math.cos(angle)
          y1 = circle_center[1] + circle_radius * math.sin(angle)
          x2 = circle_center[0] + circle_radius * math.cos(next_angle)
          y2 = circle_center[1] + circle_radius * math.sin(next_angle)

          self.sim.addObstacle([(x1, y1), (x2, y2)])
  self.sim.processObstacles()

