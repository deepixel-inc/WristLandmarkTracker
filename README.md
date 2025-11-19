# **WristLandmarkTracker**

[![Deepixel](https://img.shields.io/badge/Deepixel-Visit%20Website-blue?style=flat-square)](https://www.deepixel.xyz)
[![Get License](https://img.shields.io/badge/Get%20License-Contact%20Us-yellow)](#license)

WristLandmarkTracker is a high-performance wrist landmark detection and tracking library built on OpenCV and DeepCore, Deepixel’s proprietary inference engine.
It loads encrypted embedded models and performs independent left- and right-wrist detection, automatically searching on the appropriate side of the image.

The system detects 32 cylindrical wrist landmarks per wrist, providing:
- High-precision wrist landmark localization (32 points arranged in a cylindrical topology)
- Left/right wrist auto-selection based on spatial location (left side → left wrist, right side → right wrist)
- Per-landmark visibility confidence
- Wrist pose estimation, returned in Euler angle format
- Bounding region extraction for each wrist
- Built-in visualization utilities for rendering cylinders, ROIs, and detected keypoints
- Python bindings for easy integration

The tracker outputs the full set of wrist landmarks and pose parameters for either the left wrist or right wrist, depending on detection results.

Below is an example landmark indexing illustration (example only):

> *(Insert your wrist landmark diagram here)*
> *(E.g., index numbers correspond to positions returned by `get_keypoints(is_right=True)`)*

---

## **Features**

* Real-time wrist landmark tracking
* Supports **left & right hands independently**
* Automatic ROI sliding window search
* Landmark visibility score
* Wrist bounding box
* Head-like pose estimation for wrist orientation
* Stabilization logic for temporal smoothing
* CPU-only — no GPU needed
* Python API included
* Supports Python 3.9–3.12

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
| Notebook CPU (Intel i7 11th Gen) | 640×480    | ~300 |
| Desktop CPU (Intel i7 11th Gen)  | 640×480    | ~420 |

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

    cap = cv2.VideoCapture(0)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        wrist.run(frame, 0.2, False)

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

run()
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
        wrist.run(img, 0.2, True)

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

Loads embedded model from encrypted memory using the license key.

---

## **Tracking**

```python
wrist.run(image, fThresh, isStill)
```

| Parameter | Description                                      |
| --------- | ------------------------------------------------ |
| `image`   | BGR numpy array                                  |
| `fThresh` | detection threshold (recommend `0.2`)            |
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

Returns `N × 2` float32 matrix:
`[[x, y], [x, y], ...]`

### **Visibility**

```python
vis = wrist.get_visibility(is_right=True)
```

Returns `1 × N` float32 vector.

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

