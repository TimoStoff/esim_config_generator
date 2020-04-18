import numpy as np
import glob
import argparse
import cv2 as cv
import os

def convert(files):
    for imgfile in files:
        img = cv.imread(imgfile, cv.IMREAD_COLOR)
        b_channel, g_channel, r_channel = cv.split(img)
        alpha_channel = np.ones(b_channel.shape, dtype=b_channel.dtype) * 255
        alpha_channel[0, 0] = 254
        img_BGRA = cv.merge((b_channel, g_channel, r_channel, alpha_channel))
        print(img_BGRA.shape)
        filepng = os.path.splitext(imgfile)[0]+".png"
        cv.imwrite(filepng, img_BGRA)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Convert JPG to PNG with alpha channel')
    parser.add_argument('--filepath', type=str, help='Path to file. If given folder, will do all files in folder')
    parser.add_argument('--recursive', type=int, help='1 for recursive convert')

    args = parser.parse_args()
    filepath = args.filepath
    if os.path.isfile(filepath):
        files = [filepath]
    else:
        files = sorted(glob.glob('{}/*.jpg'.format(filepath)))
    print("Converting {}".format(files))
    convert(files)

