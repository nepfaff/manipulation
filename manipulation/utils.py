import os


def FindResource(filename):
    return os.path.join(os.path.dirname(__file__), filename)
