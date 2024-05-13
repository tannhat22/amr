import uuid

def generate_task_id():
    # Tạo một UUID ngẫu nhiên
    new_uuid = uuid.uuid4()
    # Lấy một phần của UUID (ví dụ: 8 ký tự đầu tiên)
    task_id = str(new_uuid)[:6]
    return str(task_id)

# Sử dụng hàm để tạo một task ID
task_id = generate_task_id()
print("Task ID:", task_id)
print(0==None)