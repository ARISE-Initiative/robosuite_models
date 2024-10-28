import os

MODEL_ASSETS_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "assets")


def robosuite_model_path_completion(path):
    return os.path.join(MODEL_ASSETS_PATH, path)
