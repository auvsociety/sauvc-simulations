# About v2_dataset
This repository contains image dataset from the AUV_v2 vehicle's front and down facing cameras. Before using the dataset on DSO, to make sure the names of the images are in the proper format, a python script is also provided in the package.


Use the script to rename the image files from `frameXXXX.jpg` to `XXXX.jpg` (where `XXXX` represents a whole number):
```
cd /path/to/v2_dataset/imgs
python rename_img.py
```
The rosbags are saved in AUV Society's Google drive: [link](https://drive.google.com/drive/folders/1ISMYu34JWEGwhlid0ABbksTwYFvw90cH?usp=sharing)

[Back to Home](./Home.md)