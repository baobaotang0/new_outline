import os,numpy,vtktool,math
from matplotlib import pyplot
from math_tools import new_plot
import cv2
def get_min_max_3d(car):
    p_min = car[0].copy()
    p_max = car[0].copy()
    for p in car:
        for i in range(3):
            if p[i] < p_min[i]:
                p_min[i] = p[i]
            if p[i] > p_max[i]:
                p_max[i] = p[i]
    return p_min, p_max

def cut_2dcar(car:list,idx:int,limit:list):
    res = []
    for p in car:
        if limit[0] <= p[idx] <= limit[1] :
            res.append([p[0],p[1]])
    return res

def pixel(car:list, pixel_size, p_min:list, p_max:list, darkest:float):
    resolution = [math.ceil((p_max[0]-p_min[0])/pixel_size), math.ceil((p_max[1]-p_min[1])/pixel_size)]
    res = numpy.array([numpy.array([0 for j in range(resolution[0]+1)], dtype = numpy.uint8)
                       for i in range(resolution[1]+1)], dtype = numpy.uint8)
    for p in car:
        i = int((p[0] - p_min[0]) / pixel_size)
        j = int((p[1]-p_min[1])/pixel_size)
        res[j][i] += 1
    for i in range(resolution[1]):
        for j in range(resolution[0]):
            if res[i][j] > darkest:
                res[i][j] = 255
            else:
                res[i][j] = int(255/darkest*res[i][j])
    return res


if __name__ == '__main__':

    folder_path = "cars/"
    car_id = os.listdir(folder_path)
    for i in car_id:
        # if i not in ["car_02.npy"]:
        #     continue
        if i.endswith("npy"):
            print(i)
            path2d = "cars/" + i
            with open(path2d, 'rb') as f_pos:
                car = list(numpy.load(f_pos))
                p_min, p_max = get_min_max_3d(car)
                mid = (p_min[2]+p_max[2])/2
                half_car = cut_2dcar(car,idx=2,limit=[mid,p_max[2]])
                p_max[1] = 0.95
                p_min[1] = 0
                half_car = cut_2dcar(half_car,idx=1,limit=[p_min[1],p_max[1]])
                # pyplot.figure(figsize=(20,10))
                # new_plot(half_car,".")
                # pyplot.axis("equal")
                # pyplot.show()
                # vtktool.vtk_show(car)
                pixel_size = 0.015
                # pyplot.figure(figsize=(20,5))
                mosaic_matrix = pixel(half_car,pixel_size,p_min, p_max, darkest=1)
                # c=pyplot.pcolormesh(mosaic_matrix, cmap ='magma')
                # pyplot.colorbar(c)
                # pyplot.axis("equal")
                # pyplot.show()

                img = mosaic_matrix

                image = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
                # img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                # ret, im = cv2.threshold(img_gray, 100, 255, cv2.THRESH_BINARY_INV)
                thresh = cv2.Canny(image, 128, 256)
                contours, hierarchy = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                img = cv2.drawContours(image, contours, -1, (0, 255, 75), thickness= 1,maxLevel=2)
                cv2.imshow('image', img)
                c = cv2.waitKey()


                # print(int((p_max[0]-p_min[0])*6),int((p_max[0]-p_min[0])*8.5))
                #
                # img = cv2.medianBlur(img, 3)
                # cimg = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)
                # circles = cv2.HoughCircles(img,cv2.HOUGH_GRADIENT,dp=1,minDist = (p_max[0]-p_min[0])*60,
                #                             param1=50,param2=15,minRadius=0,maxRadius=0)
                # print(circles)
                # if circles is not None:
                #     circles = numpy.uint16(numpy.around(circles))
                #     for i in circles[0,:]:
                #         # draw the outer circle
                #         cv2.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
                #         # draw the center of the circle
                #         cv2.circle(cimg,(i[0],i[1]),2,(0,0,255),3)
                # cv2.imshow('detected circles',cimg)
                # cv2.waitKey(0)
                # cv2.destroyA00llWindows()
