from typing import TypedDict, List
import numpy as np

class Facial_area(TypedDict):
    x: int
    y: int
    w: int
    h: int

class DetectionResult(TypedDict):
    face: np.ndarray
    facial_area: Facial_area
    confidence: float