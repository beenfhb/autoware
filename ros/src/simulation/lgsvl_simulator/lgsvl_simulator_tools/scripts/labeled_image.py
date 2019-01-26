#!/usr/bin/env python
import cv2

class LabeledImage:
    def __init__(self,category_number,image_dir):
        self.image_dir = image_dir
        self.category_number = category_number
        self.count = 0
        self.rois = []
    def addROI(self,roi):
        self.rois.append(roi)
    def writeROIandImage(self,image):
        height, width, color = image.shape
        if roi.detections == 0:
            return
        cv2.imwrite(self.image_dir+"/"+str(self.count)+".jpg", image)
        label_str = ""
        i = 0
        for roi in self.rois:
            if i != len(self.rois):
                label_str = label_str + str(self.category_number) + " " + str(roi.bbox.x/width) + " " + str(roi.bbox.y/height) + " " + str(roi.bbox.width/width) + " " + str(roi.bbox.height/height)
            else:
                label_str = label_str + str(self.category_number) + " " + str(roi.bbox.x/width) + " " + str(roi.bbox.y/height) + " " + str(roi.bbox.width/width) + " " + str(roi.bbox.height/height) + "\n"
        self.count = self.count + 1
        self.rois = []

if __name__ == "__main__":
    pass