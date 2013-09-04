#!/usr/bin/python

import ccv
import cv2
import numpy as np
import sys, os
from ctypes import *

if __name__ == "__main__":

    img = cv2.imread(sys.argv[1], cv2.CV_LOAD_IMAGE_GRAYSCALE)
    #img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    image = ccv.np_to_ccv(img)

    ccv.ccv_enable_default_cache()

    pimage2 = c_char_p()
    ccv.ipp_resize(image, cast(byref(pimage2), POINTER(POINTER(ccv.ccv_dense_matrix_t))))
    image2 = cast(pimage2, POINTER(ccv.ccv_dense_matrix_t))


    pimage3 = c_char_p()
    ccv.ipp_gauss_filter(image2, cast(byref(pimage3), POINTER(POINTER(ccv.ccv_dense_matrix_t))))
    image3 = cast(pimage3, POINTER(ccv.ccv_dense_matrix_t))

    zero = c_int(0)
    ccv.ccv_write(image3, "resized_py.png", byref(zero), ccv.CCV_IO_PNG_FILE, 0);

    params = ccv.ccv_swt_param_t()
    params.interval = 1

    textline_thresh = (c_double * 2)(0.1, 0.8)
    params.same_textline_thresh = textline_thresh

    params.min_neighbors = 5
    params.scale_invariant = 0
    params.size = 3
    params.low_thresh = 50
    params.high_thresh = 100
    params.max_height = 200
    params.min_height = 8
    params.min_area = 38
    params.letter_occlude_thresh = 3
    params.aspect_ratio = 8
    params.std_ratio = 0.83
    params.thickness_ratio = 3.
    params.height_ratio = 3.
    params.intensity_thresh = 41
    params.distance_ratio = 2.9
    params.intersect_ratio = 1.9
    params.letter_thresh = 3
    params.elongate_ratio = 1.9
    params.breakdown = 0
    params.breakdown_ratio = 1.0

    textlines = ccv.ccv_swt_detect_textlines2(image3, params)

    for i in range(textlines.contents.rnum):
        textline = cast(ccv.ccv_array_get(textlines, i), POINTER(ccv.ccv_textline2_t)).contents
        print ( "%d %d %d %d" % (
            textline.rect.x,
            textline.rect.y,
            textline.rect.width,
            textline.rect.height,
            )
        )

        rect = textline.rect
       # box_rect = map(int, [rect.x, rect.y, rect.width, rect.height])
       # box = Box(box_rect)

        letters = textline.letters
        for i in range(letters.contents.rnum):
            letter = cast(ccv.ccv_array_get(letters, i), POINTER(ccv.ccv_letter_t)).contents
            print ( "\t%d %d %d %d" % (
                letter.rect.x,
                letter.rect.y,
                letter.rect.width,
                letter.rect.height,
                )
            )
            #rect = letter.rect
            #letter_rect = map(int, [rect.x, rect.y, rect.width, rect.height])
            #box.add_letter(letter_rect)

        #box.filter_letters()
        #find_plate(img, box)

    ccv.ccv_matrix_free(image2)
    ccv.ccv_matrix_free(image3)
