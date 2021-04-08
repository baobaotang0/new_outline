import numpy
import vtkmodules.all as vtk
m1 = [[1, 0, 0, 2],
      [0, 3, 3, 0],
      [0, 1, 1, 1.65],
      [0, 0, 0, 1]]

m2 = [[1, 0, 0, 3.76389],
      [0, 3, 3, 0],
      [1, 0, 1, 0],
      [0, 0, 0, 1]]

m3 = [[1, 0, 0, 0],
      [0, 3, 0, 0],
      [0, 0, 1, -0.0603535],
      [5, 0, 0, 1]]


if __name__ == '__main__':

    n1 = numpy.array(m1)
    n2 = numpy.array(m2)

    n3 = n1.dot(n2)
    print(n3)
    a = vtk.vtkActor()
    a.SetOrientation(0, 0, 30)
    print(a.GetMatrix())