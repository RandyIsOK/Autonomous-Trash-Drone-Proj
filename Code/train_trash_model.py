from ultralytics import YOLO

# ---------- Dataset ----------
DATA_CONFIG = "./taco_dataset/data.yaml"  

# ---------- Base model to fine-tune from ----------
BASE_MODEL = "yolov8n.pt"

EPOCHS = 100
IMAGE_SIZE = 640
BATCH_SIZE = 16  # lower this if training runs out of memory


def train():
    model = YOLO(BASE_MODEL)

    results = model.train(
        data=DATA_CONFIG,
        epochs=EPOCHS,
        imgsz=IMAGE_SIZE,
        batch=BATCH_SIZE,
        name="trash_detector"
    )

    return model


def validate(model):
    metrics = model.val()
    print(f"mAP50-95: {metrics.box.map:.3f}")
    print(f"mAP50:    {metrics.box.map50:.3f}")


def export_for_pi(model, format="ncnn"):
    model.export(format=format)
    print(f"Exported model in {format} format")


if __name__ == "__main__":
    trained_model = train()
    validate(trained_model)
    export_for_pi(trained_model)