import matplotlib.pyplot as plt
import numpy as np


def point_to_segment_path_distance(x, y, pos_x, pos_y, dest_x, dest_y):
    px = dest_x - pos_x
    py = dest_y - pos_y
    norm = px * px + py * py

    if norm == 0:  # Nếu đoạn thẳng có độ dài bằng 0
        return ((x - pos_x) ** 2 + (y - pos_y) ** 2) ** 0.5

    u = ((x - pos_x) * px + (y - pos_y) * py) / norm
    u = max(0, min(1, u))  # Giới hạn u trong [0, 1]

    # Nếu vị trí đến vật cản nằm phía sau vị trí robot hiện tại so với hướng đường đi hiện tại thì bỏ qua vật cản
    if u == 0:
        return [(pos_x, pos_y), 100.0]

    closest_x = pos_x + u * px
    closest_y = pos_y + u * py

    return [(closest_x, closest_y), ((x - closest_x) ** 2 + (y - closest_y) ** 2) ** 0.5]


def get_look_ahead_point(start, destination, look_ahead_distance):
    # Tính vector hướng
    dx, dy = destination[0] - start[0], destination[1] - start[1]
    length = np.sqrt(dx**2 + dy**2)

    if length == 0:  # Nếu robot đã ở đích
        return start

    # Tính điểm giới hạn khoảng nhìn trước
    scale = min(look_ahead_distance / length, 1)
    look_ahead_point = (start[0] + scale * dx, start[1] + scale * dy)

    return look_ahead_point


def plot_robots_and_paths(robot_positions, destinations, radii, look_ahead_distance=3.0):
    fig, ax = plt.subplots(figsize=(10, 8))

    # Vẽ robot và đường đi
    for i, (pos, dest, radius) in enumerate(zip(robot_positions, destinations, radii)):
        x1, y1 = pos
        x2, y2 = dest

        # Tính điểm giới hạn khoảng nhìn trước
        look_ahead_point = get_look_ahead_point(pos, dest, look_ahead_distance)

        # Vẽ robot
        circle = plt.Circle((x1, y1), radius, color=f"C{i}", alpha=0.3)
        ax.scatter(x1, y1, label=f"Robot {i + 1}", c=f"C{i}")
        ax.add_artist(circle)

        # Vẽ đường đến điểm giới hạn khoảng nhìn trước
        ax.plot([x1, x2], [y1, y2], "--", color=f"C{i}")

        ax.arrow(
            x1,
            y1,
            look_ahead_point[0] - x1,
            look_ahead_point[1] - y1,
            head_width=0.5,
            head_length=0.7,
            length_includes_head=True,
            fc="red",
            ec="red",
            # label="look_ahead_dist",
        )

        # Vẽ vị trí đích đến và chú thích tọa độ
        ax.plot(x2, y2, "x", color=f"C{i}", markersize=10, label=f"Destination {i + 1}")
        ax.text(x2, y2, f"({x2}, {y2})", fontsize=9, color=f"C{i}", ha="left", va="bottom")

    # Kiểm tra xung đột
    for i, (pos1, dest1, radius1) in enumerate(zip(robot_positions, destinations, radii)):
        for j, (pos2, dest2, radius2) in enumerate(zip(robot_positions, destinations, radii)):
            if i == j:
                continue

            # Tính điểm giới hạn khoảng nhìn trước của robot i
            look_ahead_point = get_look_ahead_point(pos1, dest1, look_ahead_distance)

            # Kiểm tra khoảng cách từ pos2 đến đoạn thẳng giữa pos1 và look_ahead_point
            closest_point, dist = point_to_segment_path_distance(
                pos2[0], pos2[1], pos1[0], pos1[1], look_ahead_point[0], look_ahead_point[1]
            )

            print(f"Distance  from robot{i+1} to robot{j+1}: {dist}")
            # Điều kiện: robot cản trở phải nằm trong vùng va chạm
            if dist < radius1 + radius2:
                print(f"Robot[{i+1}] pause for detect collision!")
                ax.plot(
                    closest_point[0],
                    closest_point[1],
                    "o",
                    color=f"red",
                    label=f"Collision {i + 1}",
                )
                # circle = plt.Circle(
                #     (closest_point[0], closest_point[1]), radius, color=f"C{i}", alpha=0.3
                # )
                # ax.add_artist(circle)
                # ax.plot([pos1[0], pos2[0]], [pos1[1], pos2[1]], "r-", label="Potential Conflict")

    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    ax.set_aspect("equal", adjustable="box")
    ax.legend()
    plt.show()


# Dữ liệu đầu vào
robot_positions = [(0, 0), (0, -1), (3, 2)]
destinations = [(5, 5), (5, 3), (6, 2)]
radii = [0.6, 0.6, 0.6]

plot_robots_and_paths(robot_positions, destinations, radii, look_ahead_distance=3.0)
