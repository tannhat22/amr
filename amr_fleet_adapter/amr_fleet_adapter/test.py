import matplotlib.pyplot as plt
import random


class RobotMonitor:
    def __init__(self):
        self.fig, self.axs = plt.subplots(2, 2, figsize=(10, 8))  # Giả sử có 4 tầng, tạo grid 2x2
        self.robot_positions = {
            "level_1": [],
            "level_2": [],
            "level_3": [],
            "level_4": [],
        }
        self.level_axes = {
            "level_1": self.axs[0, 0],
            "level_2": self.axs[0, 1],
            "level_3": self.axs[1, 0],
            "level_4": self.axs[1, 1],
        }
        for level, ax in self.level_axes.items():
            ax.set_xlim(-100, 100)
            ax.set_ylim(-100, 100)
            ax.set_title(f"Level: {level}")
            ax.set_xlabel("X")
            ax.set_ylabel("Y")
            ax.grid(True)

    def update_positions(self, robot_states):
        """
        Cập nhật vị trí robot dựa trên dữ liệu mới.
        robot_states: dict chứa thông tin robot {robot_name: (level, (x, y))}
        """
        for level in self.robot_positions:
            self.robot_positions[level] = []  # Reset dữ liệu

        # Phân loại robot theo tầng
        for robot_name, (level, position) in robot_states.items():
            if level in self.robot_positions:
                self.robot_positions[level].append((robot_name, position))

        # Cập nhật đồ thị
        for level, robots in self.robot_positions.items():
            ax = self.level_axes[level]
            ax.clear()  # Xóa dữ liệu cũ
            ax.set_xlim(-100, 100)
            ax.set_ylim(-100, 100)
            ax.set_title(f"Level: {level}")
            ax.set_xlabel("X")
            ax.set_ylabel("Y")
            ax.grid(True)
            for robot_name, (x, y) in robots:
                ax.plot(x, y, "o", label=robot_name)
            if robots:
                ax.legend()

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


# Giả lập dữ liệu robot và cập nhật
monitor = RobotMonitor()

# Mô phỏng dữ liệu
import time

robot_states = {
    "robot1": ("level_1", (random.randint(-100, 100), random.randint(-100, 100))),
    "robot2": ("level_2", (random.randint(-100, 100), random.randint(-100, 100))),
    "robot3": ("level_3", (random.randint(-100, 100), random.randint(-100, 100))),
    "robot4": ("level_4", (random.randint(-100, 100), random.randint(-100, 100))),
}

plt.ion()  # Chế độ vẽ tương tác
while True:
    # Cập nhật dữ liệu mô phỏng
    for robot in robot_states:
        level, (x, y) = robot_states[robot]
        robot_states[robot] = (
            level,
            (random.randint(-100, 100), random.randint(-100, 100)),
        )
    monitor.update_positions(robot_states)
    time.sleep(1)
