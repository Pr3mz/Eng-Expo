import cv2
from inference_sdk import InferenceHTTPClient, InferenceConfiguration
client = InferenceHTTPClient(
    api_url="https://serverless.roboflow.com",
    api_key="X5wKaATp2EknpnzoIAqF",
).configure(InferenceConfiguration(api_key_transport="header"))
img = cv2.imread("dummy.jpg")
if img is None:
    img = __import__("numpy").zeros((480, 640, 3), dtype="uint8")
res = client.run_workflow(
    workspace_name="prem-supthaksina",
    workflow_id="arena-gem-and-drop-zone-detectio",
    images={"image": img}
)
print(res)
