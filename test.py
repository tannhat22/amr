import re

def search_mode_docking(dock_name: str):
    match = re.search(r'--(.+)',dock_name)
    if match:
        result = match.group(1)
        return result
    else:
        return None
a = "charger001--charge"
b = a.split("--")[0]
print(b)
print(search_mode_docking("charger001--charge"))