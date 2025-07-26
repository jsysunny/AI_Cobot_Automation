import ultralytics

model = ultralytics.YOLO("yolo11s.pt")

model.train(data='/home/hongha/YOLO_Datasets/250603_shelf/data.yaml', epochs=200, imgsz=640, batch=16)
