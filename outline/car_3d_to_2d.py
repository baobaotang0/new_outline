import os
import numpy
import xpcl

def transfer_car(folder_path: str, *args):
    a = os.listdir(folder_path)
    for car_id in a:
        string = os.path.join(folder_path, car_id)
        print("car_name", car_id)
        car = numpy.load(string)
        print(xpcl.compute_line.__doc__)
        lines = xpcl.compute_line(car, 0.25, 0.12, 2.3)[0]
        numpy.save("2dcars/2d"+car_id, lines[0])

if __name__ == '__main__':
    path = 'cars'
    transfer_car(path)