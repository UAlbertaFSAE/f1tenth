from ultralytics import YOLO
import cv2
import numpy as np
import pyzed.sl as sl

try:
    import torch
    TORCH_AVAILABLE = True
except Exception:
    TORCH_AVAILABLE = False


def main():
    model_path = 'src/zed-ros2-wrapper/detection_camera/models/model.pt'
    classes_path = 'src/zed-ros2-wrapper/detection_camera/models/classes.txt'

    # Select device
    device = 'cpu'
    if TORCH_AVAILABLE:
        try:
            if torch.cuda.is_available():
                device = 'cuda:0'
        except Exception:
            device = 'cpu'
    print(f"Using device: {device}")

    # Load model and classes
    model = YOLO(model_path)
    try:
        # Prefer explicit device for predict
        model.to(device)
    except Exception:
        pass
    with open(classes_path, 'r') as f:
        class_names = [line.strip() for line in f if line.strip()]

    # Open ZED 2i
    cam = sl.Camera()
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720
    init_params.camera_fps = 30
    init_params.depth_mode = sl.DEPTH_MODE.ULTRA  # depth not strictly needed for 2D detect
    init_params.camera_disable_self_calib = False
    status = cam.open(init_params)
    if status != sl.ERROR_CODE.SUCCESS:
        print(f"Error opening ZED camera: {status}")
        return

    runtime = sl.RuntimeParameters()
    image_mat = sl.Mat()

    win_name = 'ZED2i Cone Detection (YOLO)'
    cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)

    try:
        while True:
            if cam.grab(runtime) != sl.ERROR_CODE.SUCCESS:
                continue

            cam.retrieve_image(image_mat, sl.VIEW.LEFT)
            frame_rgba = image_mat.get_data()  # HxWx4 uint8
            # Convert RGBA->BGR for OpenCV/YOLO
            frame_bgr = cv2.cvtColor(frame_rgba, cv2.COLOR_RGBA2BGR)

            # Inference with graceful CUDA fallback
            try:
                results = model.predict(frame_bgr, conf=0.5, device=device, verbose=False)
            except Exception as e:
                msg = str(e).lower()
                if 'no kernel image' in msg or 'cuda error' in msg:
                    print('CUDA execution failed; falling back to CPU for YOLO inference...')
                    device = 'cpu'
                    try:
                        model.to('cpu')
                    except Exception:
                        pass
                    results = model.predict(frame_bgr, conf=0.5, device='cpu', verbose=False)
                else:
                    raise

            r = results[0]

            # Draw detections
            if r.boxes is not None and len(r.boxes) > 0:
                for box in r.boxes:
                    xyxy = box.xyxy[0].tolist()
                    x1, y1, x2, y2 = map(int, xyxy)
                    cls_id = int(box.cls[0].item()) if hasattr(box.cls[0], 'item') else int(box.cls[0])
                    conf = float(box.conf[0].item()) if hasattr(box.conf[0], 'item') else float(box.conf[0])

                    label = class_names[cls_id] if 0 <= cls_id < len(class_names) else str(cls_id)
                    text = f"{label} {conf:.2f}"
                    cv2.rectangle(frame_bgr, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(frame_bgr, text, (x1, max(0, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            cv2.imshow(win_name, frame_bgr)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        cam.close()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()