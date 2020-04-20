import numpy as np
import glob
import argparse
import cv2 as cv
import os

def convert(files, output_dir):
    for img_jpg in files:
        img = cv.imread(img_jpg, cv.IMREAD_COLOR)
        b_channel, g_channel, r_channel = cv.split(img)
        alpha_channel = np.ones(b_channel.shape, dtype=b_channel.dtype) * 255
        alpha_channel[0, 0] = 254
        img_BGRA = cv.merge((b_channel, g_channel, r_channel, alpha_channel))
        img_name = os.path.splitext(os.path.basename(img_jpg))[0]+".png"
        img_png = os.path.join(output_dir, img_name)
        print("Converting {} to {}, size={}".format(img_jpg, img_png, img_BGRA.shape))
        cv.imwrite(img_png, img_BGRA)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Convert JPG to PNG with alpha channel')
    parser.add_argument('filepath', type=str, help='Path to file. If given folder,\
            will do all jpg files in folder')
    parser.add_argument('--output_dir', type=str, default=None, help='Output save dir.\
            If left empty, same as input dir')

    args = parser.parse_args()
    filepath = args.filepath
    output_dir = args.output_dir
    if os.path.isfile(filepath):
        files = [filepath]
        if args.output_dir is None:
            output_dir = os.path.dirname(args.filepath)
    else:
        files = sorted(glob.glob('{}/*.jpg'.format(filepath)))
        if len(files)>0 and args.output_dir is None:
            output_dir = os.path.dirname(files[0])
    os.makedirs(output_dir, exist_ok=True)
    convert(files, output_dir)

