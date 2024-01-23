# poses = [
#     [1.0, 2.0, 0.0],
#     [3.0, 4.0, 1.57],
#     # ... other poses ...
# ]
# data = {}
# data['waypoints'] = [{'x': pose[0], 'y': pose[1], 'yaw': pose[2]} for pose in poses]
# print(data['waypoints'])
# a = None
# print(a == False)

# import re

# chuoi = "TTR_line55"

# # Sử dụng biểu thức chính quy (regex) để lấy chuỗi bắt đầu từ sau kí tự --
# match = re.search(r'--(.+)', chuoi)

# if match:
#     ket_qua = match.group(1)
#     print(ket_qua)
# else:
#     print("Không tìm thấy chuỗi phù hợp.")

# def search_mode_docking(dock_name: str):
#     match = re.search(r'--(.+)',dock_name)
#     if match:
#         result = match.group(1)
#         return result
#     else:
#         return None

# print(search_mode_docking(chuoi))
import enum

a = {'a': 1, 'b': 2, 'c': 3}
print(a.get('a'))
for i in a:
    if i == 'a':
        continue
    print(i)

