from typing import TypedDict, List
import numpy as np

class FaceRecognitionRow(TypedDict):
    identity: str
    face: np.ndarray
    #VGG-Face_cosine: float
    #problem with "-"
    VGG_Face_cosine: float
    source_x: int
    source_y: int
    source_w: int
    source_h: int
