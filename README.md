# RoboMaster Line and Marker Tracking Project

This project involves the development of a RoboMaster robot that can track lines, detect markers, and execute specific tasks based on the detected markers. The project is implemented in Python using the RoboMaster SDK.

## Table of Contents

- [Features](#features)
- [Requirements](#requirements)
- [Installation](#installation)
- [Usage](#usage)
- [Code Explanation](#code-explanation)
- [Contributing](#contributing)
- [License](#license)

## Features

- **Line Tracking**: The robot can follow a line using vision-based detection.
- **Marker Detection**: The robot can detect and respond to markers.
- **Task Execution**: The robot performs specific tasks based on the detected markers.
- **Obstacle Detection**: The robot can detect obstacles using distance sensors and stop accordingly.

## Requirements

- Python 3.6+
- RoboMaster SDK
- OpenCV
- NumPy

## Installation

1. **Clone the repository**:

   \`\`\`bash
   git clone https://github.com/yourusername/robomaster-line-tracking.git
   cd robomaster-line-tracking
   \`\`\`

2. **Install the required packages**:

   \`\`\`bash
   pip install -r requirements.txt
   \`\`\`

## Usage

1. **Connect the RoboMaster robot** to your network and note the serial numbers (SN) of your robots.

2. **Update the serial numbers** in the \`main()\` function:

   \`\`\`python
   robot1.initialize(conn_type="sta", sn="YOUR_ROBOT_1_SN")
   robot2.initialize(conn_type="sta", sn="YOUR_ROBOT_2_SN")
   \`\`\`

3. **Run the script**:

   \`\`\`bash
   python your_script.py
   \`\`\`

## Code Explanation

### MarkerInfo Class

This class stores information about detected markers, including position and size.

\`\`\`python
class MarkerInfo:
    def __init__(self, x, y, w, h, info):
        self.x = x
        self.y = y
        self.w = w
        self.h = h
        self.info = info
\`\`\`

### PointInfo Class

This class stores information about detected points on the line, including position and angle.

\`\`\`python
class PointInfo:
    def __init__(self, x, y, theta, c):
        self.x = x
        self.y = y
        self.theta = theta
        self.c = c
\`\`\`

### PIDController Class

This class implements a PID controller for controlling the robot's movements.

\`\`\`python
class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, setpoint, current_value):
        error = current_value - setpoint
        self.integral += error
        derivative = error - self.prev_error
        self.prev_error = error
        return self.Kp * error + self.Ki * self.integral + self.Kd * derivative
\`\`\`

### Main Functions

- \`on_detect_marker()\`: Callback for marker detection.
- \`on_detect_line()\`: Callback for line detection.
- \`sub_data_distance()\`: Callback for distance sensor data.
- \`configure_robot()\`: Configures the robot components.
- \`track_line()\`: Main function to track lines and handle markers.
- \`execute_task()\`: Executes specific tasks based on detected markers.
- \`catch_and_return()\`: Grips an object and returns to the starting point.

### Main Execution

The \`main()\` function initializes two robots, sets up their tasks, and executes them in sequence.

\`\`\`python
def main():
    try:
        # Initialize robots
        robot1 = robot.Robot()
        robot1.initialize(conn_type="sta", sn="YOUR_ROBOT_1_SN")
        robot2 = robot.Robot()
        robot2.initialize(conn_type="sta", sn="YOUR_ROBOT_2_SN")

        # Execute tasks
        track_line(robot1, 1)
        execute_task(robot2, markers[0].info if markers else None)
        track_line(robot2, 2)
        catch_and_return(robot2)
        track_line(robot1, 1)
        logging.info("The End")

    except KeyboardInterrupt:
        logging.warning("Emergency stop!")

    finally:
        robot1.close()
        robot2.close()
        logging.info("All robots closed")
\`\`\`

## Contributing

Contributions are welcome! Please fork the repository and submit a pull request with your changes.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for more details.
