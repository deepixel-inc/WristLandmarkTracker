# **WristLandmarkTracker**

[![Deepixel](https://img.shields.io/badge/Deepixel-Visit%20Website-blue?style=flat-square)](https://www.deepixel.xyz)
[![Get License](https://img.shields.io/badge/Get%20License-Contact%20Us-yellow)](#license)

WristLandmarkTracker is a high-performance wrist landmark detection and tracking library built on OpenCV and DeepCore, Deepixel’s proprietary inference engine. It loads encrypted embedded models and performs independent detection for left and right wrists, automatically searching the correct side of the image during initialization.

The system detects 8 cylindrical wrist landmarks per wrist and provides:

* High-precision 3D landmark localization (cylindrical topology)
* Left/right wrist detection with per-landmark visibility confidence
* Wrist pose estimation in Euler angles (pitch, yaw, roll)

Below is an example of a landmark detection result. The 3D cylinder structure, along with the orientation and pose of the corresponding wrist, is printed on the bounding rect's top left corner.

<img width="680" height="455" alt="image" src="https://github.com/user-attachments/assets/ba563e02-e550-41f1-a319-665cec9b6832" />

---

### Features ###

* Real-time wrist landmark tracking
* Independent support for left and right wrists
* Automatic ROI sliding window search (initial assignment only)
* Landmark visibility scores
* Wrist bounding boxes and pose estimation
* Temporal smoothing for stabilized tracking
* CPU-only operation (no GPU required)
* Built-in visualization utilities
* Python API (supports Python 3.9–3.12)

---

### 📌 3D Cylindrical Keypoint Coordinates

The following tables show the normalized 3D coordinates of the eight cylindrical keypoints used to model the wrist-fitting structure.
Each keypoint represents a vertex around the cylindrical surface.

#### **🧭 Left Wrist Structure**

```
pts0 = { -1, -1,  0 }
pts1 = {  0, -1, -1 }
pts2 = {  1, -1,  0 }
pts3 = {  0, -1,  1 }
pts4 = { -1,  1,  0 }
pts5 = {  0,  1, -1 }
pts6 = {  1,  1,  0 }
pts7 = {  0,  1,  1 }
```

#### **🧭 Right Wrist Structure**

```
pts0 = {  1, -1,  0 }
pts1 = {  0, -1, -1 }
pts2 = { -1, -1,  0 }
pts3 = {  0, -1,  1 }
pts4 = {  1,  1,  0 }
pts5 = {  0,  1, -1 }
pts6 = { -1,  1,  0 }
pts7 = {  0,  1,  1 }
```

Below is the 3D cylindrical structure fitted to the user’s wrist, with the keypoint indices labeled next to each point.

<img width="644" height="278" alt="image" src="https://github.com/user-attachments/assets/1ba38756-4f65-4a73-88ef-de3a3e998b6c" />

The symmetry in the point index ensures both cylinders share the same geometric topology while maintaining correct orientation on the user's left and right wrists.

---

## **Installation**

Install using the `.whl` that matches your Python version:

```bash
# Python 3.10
pip install deeppy-wrist-2.19.459-cp310-cp310-win_amd64.whl

# Python 3.11
pip install deeppy-wrist-2.19.459-cp311-cp311-win_amd64.whl
```

---

## **Performance**

`WristLandmarkTracker` is optimized for real-time CPU usage.

| Environment                      | Resolution | FPS  |
| -------------------------------- | ---------- | ---- |
| Notebook CPU (Intel i7 11th Gen) | 640×480    | ~200 |
| Desktop CPU (Intel i7 11th Gen)  | 640×480    | ~280 |

Performance may vary depending on:
✔ input resolution
✔ lighting
✔ wrist size in frame
✔ whether debug drawing is enabled

---

## **Python Usage**

### **1. Process Live Webcam Stream**

```python
import cv2
from deeppy import WristLandmarkTracker

license_path = "dp_wrist_2025.lic"

def run():
    wrist = WristLandmarkTracker()
    wrist.init(license_path)

    cap = cv2.VideoCapture(0,cv2.CAP_DSHOW) # use DSHOW for windows

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        wrist.run(frame, 0.75, False)

        kp_right = wrist.get_keypoints(True)
        kp_left  = wrist.get_keypoints(False)

        rect_right = wrist.get_rect(True)
        pose_right = wrist.get_pose(True)

        dbg = wrist.display_debug(frame)
        cv2.imshow("WristLandmarkTracker", dbg)

        if cv2.waitKey(1) & 0xFF == 27:
            break

    cap.release()
    cv2.destroyAllWindows()

```

---

### **2. Process Images**

```python
import cv2
from deeppy import WristLandmarkTracker

license_path = "dp_wrist_2025.lic"

def run_imgs(img_list):
    wrist = WristLandmarkTracker()
    wrist.init(license_path)

    for path in img_list:
        img = cv2.imread(path)
        wrist.run(img, 0.75, True)

        print("Right Wrist Keypoints:", wrist.get_keypoints(True))
        print("Left Wrist Keypoints:", wrist.get_keypoints(False))

        dbg = wrist.display_debug(img)
        cv2.imshow("WristLandmarkTracker", dbg)

        if cv2.waitKey(0) & 0xFF == 27:
            break

    cv2.destroyAllWindows()
```

---

## **API Reference**

### **Initialization**

```python
wrist = WristLandmarkTracker()
success = wrist.init("dp_wrist_2025.lic")
```

Loads the embedded model from encrypted memory using the license key.

---

## **Tracking**

```python
wrist.run(image, fThresh, isStill)
```

| Parameter | Description                                      |
| --------- | ------------------------------------------------ |
| `image`   | BGR numpy array                                  |
| `fThresh` | detection threshold (recommend `0.75`)            |
| `isStill` | whether image is static (improves stabilization) |

Internally performs:

* Sliding window ROI search
* Landmark estimation
* Gaussian temporal stabilization
* Pose estimation (rotation → Euler)

---

## **Get Results**

### **Keypoints**

```python
kps = wrist.get_keypoints(is_right=True)
```

Returns `8 × 2` float32 matrix:
`[[x, y], [x, y], ...]`

### **Visibility**

```python
vis = wrist.get_visibility(is_right=True)
```

Returns `1 × 8` float32 vector.

### **Bounding Box**

```python
rect = wrist.get_rect(is_right=True)
```

Returns **[x, y, w, h]** centered around wrist joint.

### **Pose**

```python
pose = wrist.get_pose(is_right=True)
```

Returns Euler rotation `[pitch, yaw, roll]`.

---

## **Visualization**

```python
out = wrist.display_debug(image)
```

Draws:

* wrist cylinder
* ROI rectangles
* keypoints

---

## **License**

This library is **proprietary** and requires a valid license.

To obtain a license:

📧 **[support@deepixel.xyz](mailto:support@deepixel.xyz)**

---

## **Image Credit**

Any sample images used in documentation should be synthetic or generated.

---

