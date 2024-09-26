import json

a = {
    "type": "dispatch_task_request",
    "request": {
        "category": "delivery",
        "description": {
            "pickup": {
                "place": "pantry",
                "handler": "coke_dispenser",
                "payload": [{"sku": "coke", "quantity": 1}],
            },
            "dropoff": {
                "place": "hardware_2",
                "handler": "coke_ingestor",
                "payload": [{"sku": "coke", "quantity": 1}],
            },
        },
    },
}
print(a["request"].get("nhat", 123))
a = "lift001-L2"
print(a[:-2])
