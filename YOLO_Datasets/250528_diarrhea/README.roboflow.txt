
pill_diarrhea - v1 2025-05-28 5:48pm
==============================

This dataset was exported via roboflow.com on May 28, 2025 at 8:48 AM GMT

Roboflow is an end-to-end computer vision platform that helps you
* collaborate with your team on computer vision projects
* collect & organize images
* understand and search unstructured image data
* annotate, and create datasets
* export, train, and deploy computer vision models
* use active learning to improve your dataset over time

For state of the art Computer Vision training notebooks you can use with this dataset,
visit https://github.com/roboflow/notebooks

To find over 100k other datasets and pre-trained models, visit https://universe.roboflow.com

The dataset includes 48 images.
Pill_diarrhea are annotated in YOLOv11 format.

The following pre-processing was applied to each image:
* Auto-orientation of pixel data (with EXIF-orientation stripping)
* Resize to 640x640 (Stretch)

The following augmentation was applied to create 3 versions of each source image:
* 50% probability of horizontal flip
* 50% probability of vertical flip
* Equal probability of one of the following 90-degree rotations: none, clockwise, counter-clockwise, upside-down
* Randomly crop between 0 and 13 percent of the image
* Random rotation of between -15 and +15 degrees
* Random shear of between -4° to +4° horizontally and -4° to +4° vertically
* Random brigthness adjustment of between -15 and +15 percent
* Salt and pepper noise was applied to 0.02 percent of pixels


