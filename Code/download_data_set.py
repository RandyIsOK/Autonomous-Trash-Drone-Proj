from roboflow import Roboflow

rf = Roboflow(api_key="2A8t6U1ZTdKFLvJyAAh9")
project = rf.workspace("mohamed-traore-2ekkp").project("taco-trash-annotations-in-context")
version = project.version(15)
dataset = version.download("yolov8", location="./taco_dataset")

print(dataset.location)