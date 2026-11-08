import argparse


def main():
    p = argparse.ArgumentParser(description='Train YOLO box detector')
    p.add_argument('--data', default='data.yaml')
    p.add_argument('--model', default='yolo11n.pt')
    p.add_argument('--epochs', type=int, default=100)
    p.add_argument('--imgsz', type=int, default=960)
    p.add_argument('--device', default='cpu')
    a = p.parse_args()
    from ultralytics import YOLO
    YOLO(a.model).train(data=a.data, epochs=a.epochs, imgsz=a.imgsz,
                        device=a.device, project='runs', name='boxes', seed=42)


if __name__ == '__main__':
    main()
