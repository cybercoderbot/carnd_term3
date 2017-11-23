# Semantic Segmentation
### Introduction
In this project, a Fully Convolutional Network (FCN) is implemented to segment the road pixels from the image. The fully convolutional network based is based on the VGG-16 image classifier and the performance is evaluated using the KITTI dataset.


## Results

Below is the animation from the output of the fully convolutional network, with the segmentation class overlaid upon the original image in green.

![aim](./result/animation.gif)

Performance is very good, but not perfect with only spots of road identified in a handful of images.

### Setup
##### Frameworks and Packages
Make sure you have the following is installed:
 - [Python 3](https://www.python.org/)
 - [TensorFlow](https://www.tensorflow.org/)
 - [NumPy](http://www.numpy.org/)
 - [SciPy](https://www.scipy.org/)
 
##### Dataset
Download the [Kitti Road dataset](http://www.cvlibs.net/datasets/kitti/eval_road.php) from [here](http://www.cvlibs.net/download.php?file=data_road.zip).  Extract the dataset in the `data` folder.  This will create the folder `data_road` with all the training a test images.


##### Run
Run the following command to run the project:
```
python main.py
```
**Note** If running this in Jupyter Notebook system messages, such as those regarding test status, may appear in the terminal rather than the notebook.

