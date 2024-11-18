import json

json_data = '[{"load": 1}, {"load": 2}]'
python_data = json.loads(json_data)
for data in python_data:
    print(data["load"])
# Output: [{'load': 1}, {'load': 2}]
