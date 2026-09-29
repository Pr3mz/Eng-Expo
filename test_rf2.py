from inference_sdk import InferenceHTTPClient, InferenceConfiguration
import urllib.request
import cv2
import numpy as np

# Download a sample image that likely has a gem or drop zone
url = "https://raw.githubusercontent.com/ultralytics/yolov5/master/data/images/zidane.jpg"
req = urllib.request.urlopen(url)
arr = np.asarray(bytearray(req.read()), dtype=np.uint8)
img = cv2.imdecode(arr, -1)

client = InferenceHTTPClient(
    api_url="https://serverless.roboflow.com",
    api_key="X5wKaATp2EknpnzoIAqF",
).configure(InferenceConfiguration(api_key_transport="header"))
res = client.run_workflow(
    workspace_name="prem-supthaksina",
    workflow_id="arena-gem-and-drop-zone-detectio",
    images={"image": img}
)
import json
print(json.dumps(res, indent=2))
