# -*- coding: utf-8 -*-
"""
Created on Mon Nov  1 15:04:37 2021

@author: Sandra Drusova
"""

import flirimageextractor
from matplotlib import cm

file = 'Samples/FLIR_20211101_025930.jpg'

flir = flirimageextractor.FlirImageExtractor(palettes=[cm.jet, cm.bwr, cm.gist_ncar])
# flir.process_image(file)
# flir.save_images()
# flir.plot()
meta=flir.get_metadata(file)

print(meta)