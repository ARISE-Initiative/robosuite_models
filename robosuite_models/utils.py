import os

MENAGERIE_ASSETS_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "assets")

def robosuite_model_path_completion(path):
    return os.path.join(MENAGERIE_ASSETS_PATH, path)