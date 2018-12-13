import os

def autoware_path():
    return os.path.abspath(os.path.join(os.path.dirname(__file__), 8 * "../"))

def package_path():
    return os.path.abspath(os.path.join(os.path.dirname(__file__), 3 * "../"))

def plugins_path():
    return os.path.join(package_path(), "plugins")

def profile_path(profile = ""):
    return os.path.join(package_path(), "profiles", profile)