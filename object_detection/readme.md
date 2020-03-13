# Train a Tensorflow model from scratch using Google Colab

Colab offers free access to a computer that has reasonable GPU, it is a cloud service based on Jupyter Notebooks and internet connectivity is required for access.

## Preprocessing

The first step is to take pictures of the objects to train, after doing that, you have to reduce the resolution of the pictures (this reduces training time) and split the images into train/test folders.

run the [transform_image_resolution.py](https://github.com/RoBorregos/Robocup-Home/tree/develop/object_detection/scripts/transform_image_resolution.py) script:
```bash
python transform_image_resolution.py -d ../images/ -s 800 600
```
- `-d` is the directory containing all the images.
- `-s` is the resolution that will be applied to the images.

run the [split_images](https://github.com/RoBorregos/Robocup-Home/tree/develop/object_detection/scripts/split_images.py) script.
```bash
python split_images.py -d ../images/complete_dataset -o ../images --train 80
```
- `-d` is the directory containing all the images.
- `-o` is the directory where the train and test folder will be created.
- `--train` is the percentage of images that will be used for training, 80% for train and %20 for test is the recommended. 

Now you need to label the images, use the [labelimg](https://github.com/tzutalin/labelImg) open source tool to label all the pictures in both train/test directories (this is a tedious and long process).

![labeling](https://github.com/RoBorregos/Robocup-Home/blob/object_detection/object_detection/images/readme_images/labeling.png)

**As of this point you should have a `image` directory that contains your train and test images with respective xml file of each images.**

Download the [generate_tfrecord.py](https://github.com/RoBorregos/Robocup-Home/tree/develop/object_detection/scripts/generate_tfrecord.py) script and change the `class_text_to_int` function by adding your own labels.
```python
def  class_text_to_int(row_label):
	if row_label ==  'powerade':
		return  1
	elif row_label ==  'chocolate':
		return  2
	elif row_label ==  'dr_pepper':
		return  3
	elif row_label ==  'danup':
		return  4
	else:
		None
```

Download the [labelmap](https://github.com/RoBorregos/Robocup-Home/blob/develop/object_detection/training/labelmap.pbtxt) file and change the `id` and `name` to your own labels. **NOTE:** Be consistent with the id you wrote on the `class_text_to_int` function.

We are using the Faster-RCNN-Inception-V2 model. [Download the model here.](http://download.tensorflow.org/models/object_detection/faster_rcnn_inception_v2_coco_2018_01_28.tar.gz) Open the downloaded faster_rcnn_inception_v2_coco_2018_01_28.tar.gz file with a file archiver and extract the faster_rcnn_inception_v2_coco_2018_01_28 folder.

Download the [faster_rcnn_inception_v2_pets](https://github.com/RoBorregos/Robocup-Home/blob/develop/object_detection/training/faster_rcnn_inception_v2_pets.config) file and change:

- Line 9. Change `num_classes` to the number of different objects you want the classifier to detect.
- Line 130. Change `num_examples` to the number of images you have in the `\images\test` directory.

## Setup Google Colab Notebook

* Create a directory in your google drive.
* Download the [train_model](https://github.com/RoBorregos/Robocup-Home/tree/develop/object_detection/train_model.ipynb) notebook from the RoBorregos @Home repo.
* Go to Colab, sign in with the same account you used to create the directory, create a new notebook and open the train_model notebook.
* In the notebook go to Runtime > Change Runtime Type and make sure to select GPU as Hardware accelerator.
* Click connect to start using the notebook

The first command is to check if you are using GPU.
```python
import tensorflow as tf

device_name = tf.test.gpu_device_name()
if device_name != '/device:GPU:0':
	raise  SystemError('GPU device not found')
print('Found GPU at: {}'.format(device_name))
```
You should see `Found GPU at: /device:GPU:0`

The second command is to mount Google Drive with the notebook, click on the link. Then sign in with your google drive account, and grant it access. you will be redirected to a page, copy the code on that page and paste it in the text-box of the Colab session you are running.

The following commands are very straightforward, just make sure to change the paths by replacing the name of your folder.
E.g:
```bash
cd /content/gdrive/My Drive/@Home/models/research/
```
To:
```bash
cd /content/gdrive/My Drive/Your_Folder/models/research/
```

After cloning the repo, you have to upload the previous files you edited to the `object_detection` folder.

- Upload the `generate_tfrecord.py` script to `your_folder/models/research/object_detection`
- Download the [xml_to_csv.py](https://github.com/RoBorregos/Robocup-Home/tree/develop/object_detection/scripts/xml_to_csv.py) script and upload it to `your_folder/models/research/object_detection`
- Create a folder in `your_folder/models/research/object_detection` named `training` and upload the `labelmap.pbtxt` and `faster_rcnn_inception_v2_pets.config` files.
- Create a folder in `your_folder/models/research/object_detection` named `images` and upload the test/train folders containing the images and xml files.
- Upload the `faster_rcnn_inception_v2_coco_2018_01_28` folder to the `object_detection` folder.

Follow the notebook and continue with the commands.

## Training 

Once you run the `xml_to_csv` and 	`generate_tfrecords` scripts, the next step is to run the `train.py` script to start the training, just keep following the notebook and after you run the train script in absence of errors, you should see and output like this:

```
INFO:tensorflow:global step 1: loss = 25.45 (5.327 sec/step)

........  
........

INFO:tensorflow:global step 1350: loss = 0.6345 (0.231 sec/step)  
INFO:tensorflow:global step 1351: loss = 0.5220 (0.332 sec/step)  
INFO:tensorflow:global step 1352: loss = 0.6718 (0.133 sec/step)  
INFO:tensorflow:global step 1353: loss = 0.6758 (0.432 sec/step)  
INFO:tensorflow:global step 1354: loss = 0.7454 (0.452 sec/step)  
INFO:tensorflow:global step 1355: loss = 0.8354 (0.323 sec/step)
```

As we are running this in Colab, with 3-4 hours its pretty enough, every 5 minutes aprox the changes are automatically saved in the `training` folder. Stop the training with `CTRL + C`.

The final step is to export your inference graph, the command is already in the notebook, the only thing you have to change is the `--trained_checkpoint_prefix` flag.  Go to the `training` folder and you will see some of the last checkpoints the model saved, copy the `id` of the last checkpoint and change it on the command:
`--trained_checkpoint_prefix training/model.ckpt-158879`

![traning_folder](https://github.com/RoBorregos/Robocup-Home/blob/object_detection/object_detection/images/readme_images/training_folder.png)

Finally, you can run the code in the last shell of the notebook to test your model, take new pictures to test the model and place them on a folder named `test_images`.

Change the `IMAGE_NAME` on the script:
```python
IMAGE_NAME = 'test_images/IMG_0681.jpg'
```

Run the shell, you should see the image with the objects detected:

![test_model](https://github.com/RoBorregos/Robocup-Home/blob/object_detection/object_detection/images/readme_images/test_model.png)
