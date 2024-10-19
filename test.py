import math


# Hàm tính tọa độ 4 điểm của hình chữ nhật dựa trên tọa độ (x, y) và yaw
def calculate_rectangle_corners(
    x_center, y_center, width, height, yaw, front_extension
):
    # Tính cosine và sine của yaw
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)

    # Tính tọa độ 4 điểm với chiều dài mở rộng phía trước
    corners = {
        "front_left": (
            x_center - (width / 2) * cos_yaw - (height / 2) * sin_yaw,
            y_center - (width / 2) * sin_yaw + (height / 2) * cos_yaw,
        ),
        "front_right": (
            x_center + (width / 2) * cos_yaw - (height / 2) * sin_yaw,
            y_center + (width / 2) * sin_yaw + (height / 2) * cos_yaw,
        ),
        "back_left": (
            x_center - (width / 2) * cos_yaw + (height / 2) * sin_yaw,
            y_center - (width / 2) * sin_yaw - (height / 2) * cos_yaw,
        ),
        "back_right": (
            x_center + (width / 2) * cos_yaw + (height / 2) * sin_yaw,
            y_center + (width / 2) * sin_yaw - (height / 2) * cos_yaw,
        ),
    }

    # Tính toán tọa độ cho các điểm phía trước với chiều dài mở rộng
    front_extension_corners = {
        "front_left": (
            corners["front_left"][0] + front_extension * cos_yaw,
            corners["front_left"][1] + front_extension * sin_yaw,
        ),
        "front_right": (
            corners["front_right"][0] + front_extension * cos_yaw,
            corners["front_right"][1] + front_extension * sin_yaw,
        ),
    }

    return {
        "front_left": front_extension_corners["front_left"],
        "front_right": front_extension_corners["front_right"],
        "back_left": corners["back_left"],
        "back_right": corners["back_right"],
    }


# Dữ liệu cho robot (tọa độ hiện tại, yaw, chiều rộng, chiều cao, chiều dài mở rộng)
x_center_robot = 5  # Tọa độ hiện tại của robot
y_center_robot = 5
yaw_robot = math.radians(30)  # Hướng yaw của robot (được tính bằng radian)
width_robot = 2  # Chiều rộng của robot
height_robot = 3  # Chiều cao của robot
front_extension = 2  # Chiều dài mở rộng phía trước

# Tính tọa độ 4 điểm của hình chữ nhật
corners = calculate_rectangle_corners(
    x_center_robot,
    y_center_robot,
    width_robot,
    height_robot,
    yaw_robot,
    front_extension,
)

# In kết quả
for key, value in corners.items():
    print(f"{key}: {value}")


def is_rectangles_overlap(rect1, rect2):
    # Lấy tọa độ của các điểm
    rect1_points = [
        rect1["front_left"],
        rect1["front_right"],
        rect1["back_left"],
        rect1["back_right"],
    ]

    rect2_points = [
        rect2["front_left"],
        rect2["front_right"],
        rect2["back_left"],
        rect2["back_right"],
    ]

    # Tìm min và max cho hình chữ nhật 1
    x1_min = min(point[0] for point in rect1_points)
    x1_max = max(point[0] for point in rect1_points)
    y1_min = min(point[1] for point in rect1_points)
    y1_max = max(point[1] for point in rect1_points)

    # Tìm min và max cho hình chữ nhật 2
    x2_min = min(point[0] for point in rect2_points)
    x2_max = max(point[0] for point in rect2_points)
    y2_min = min(point[1] for point in rect2_points)
    y2_max = max(point[1] for point in rect2_points)

    # Kiểm tra điều kiện không chồng lấn
    if x1_max < x2_min or x2_max < x1_min or y1_max < y2_min or y2_max < y1_min:
        return False  # Không chồng lấn

    return True  # Có chồng lấn


# Ví dụ về hai hình chữ nhật
rectangle1 = {
    "front_left": (1, 1),
    "front_right": (3, 1),
    "back_left": (1, 4),
    "back_right": (3, 4),
}

rectangle2 = {
    "front_left": (2, 2),
    "front_right": (4, 2),
    "back_left": (2, 5),
    "back_right": (4, 5),
}

# Kiểm tra chồng lấn
if is_rectangles_overlap(rectangle1, rectangle2):
    print("Hai hình chữ nhật có chồng lấn.")
else:
    print("Hai hình chữ nhật không chồng lấn.")
