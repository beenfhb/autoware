import os

def autoware():
    return os.path.abspath(os.path.join(os.path.dirname(__file__), 8 * "../"))

def package():
    return os.path.abspath(os.path.join(os.path.dirname(__file__), 3 * "../"))

def plugins():
    return os.path.join(package(), "plugins")

def profile(profile = ""):
    return os.path.join(package(), "profiles", profile)

def makedirs(path, mode=0777, exist_ok=False): # workaround in python2
    if not (exist_ok and os.path.exists(path)): os.makedirs(path, mode)