import face_recognition_models
from PIL import Image
import sys
import os

path_dir = "./AMLO/"
known_image = "./AMLO/1.jpg"
known_persons = {}
unknown_image = "./AMLO/2.jpg"

def findAllFaces():
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

def registerNewPerson():
    global known_persons
    
    newName = sys.argv[1]
    newEncoding = getEncoding()

    # If newName doesn't exists as a key value, add it to the dictionary
    if newName not in known_persons:
        known_persons[newName] = newEncoding

def getEncoding():
    global known_image

    img = face_recognition.load_image_file(known_image)
    img_encoding = face_recognition.face_encodings(img)[0]

    return img_encoding

def searchForKnownFaces():
    global known_persons
    global unknown_image

    img2search = face_recognition.load_image_file(unknown_image)
    encoding_img2search = face_recognition.face_encodings(img2search)[0]

    for key, value in known_persons.items():
        print(face_recognition.compare_faces(value, encoding_img2search), key)

if __name__ == "-__main__":
    registerNewPerson()
    searchForKnownFaces()

