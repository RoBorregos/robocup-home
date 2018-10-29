import face_recognition
from PIL import Image
import sys
import os

path_dir = "./AMLO/"
for f in os.listdir(path_dir):
    if f.endswith(".jpg"):
        img_name = path_dir + f
        img = face_recognition.load_image_file(img_name)
        # Search for faces in img, return a list of found items
        face_locations = face_recognition.face_locations(img)

        print("I found {} faces in this photograph.".format(len(face_locations)))

        for face_location in face_locations:
            top, right, bottom, left = face_location

            face_image = img[top:bottom, left:right]
            pil_image = Image.fromarray(face_image)
            pil_image.show()
