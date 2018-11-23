#!/usr/bin/env python
import numpy as np
import cv2

class Shadow:
    sigma_list = [15, 80, 250]
    G = 5.0
    b = 25.0
    alpha = 125.0
    beta = 46.0
    low_clip = 0.01
    high_clip = 0.99
    def __init__(self):
        return
    
    def shadowRemoving(self, src):
        retinex = self.Retinex(src)
        src = retinex.MSRCR(src ,self.sigma_list, self.G, self.b, self.alpha, 
                                self.beta, self.low_clip, self.high_clip)
        cv2.imshow('test shadow', src)
        return

    class Retinex:
        def __init__(self, src):
            self.src = src

        def singleScaleRetinex(self, img, sigma):
            retinex = np.log10(img) - np.log10(cv2.GaussianBlur(img, (0, 0), sigma))
            return retinex

        def multiScaleRetinex(self, img, sigma_list):
            retinex = np.zeros_like(img)
            # for sigma in sigma_list:
            retinex += 3 * np.log10(img)

            retinex = retinex / len(sigma_list)
            return retinex
    
        def MSRCR(self, src, sigma_list, G, b, alpha, beta, low_clip, high_clip):
            img = np.float64(src) + 1.0
            retinex = np.zeros_like(img)
            retinex += 3 * np.log10(img)
            retinex = retinex / len(sigma_list)

            # img_retinex = self.multiScaleRetinex(img, sigma_list)
            return retinex
