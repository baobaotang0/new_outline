import numpy
import vtkmodules.all as vtk
from math import cos, sin, radians, tan, sqrt
from vtk.util.numpy_support import vtk_to_numpy, numpy_to_vtk

import numpy
import vtkmodules.all as vtk
from vtk.util.numpy_support import numpy_to_vtk


class Cloud(object):
    def __init__(self):
        """初始化变量"""
        self.cloud_vtk_points = vtk.vtkPoints()
        self.cloud_vtk_polydata = vtk.vtkPolyData()
        self.cloud_octree = vtk.vtkOctreePointLocator()  # 外部的octree
        self.cloud_color = vtk.vtkUnsignedCharArray()
        self.cloud_actor = vtk.vtkActor()
        self.v_filter = vtk.vtkVertexGlyphFilter()
        self.mapper = vtk.vtkPolyDataMapper()

    def initialize_car(self, path):
        """载入点云数据"""
        self.np_cloud = numpy.load(path)
        self.cloud_vtk_points.SetData(numpy_to_vtk(self.np_cloud))
        self.cloud_vtk_polydata.SetPoints(self.cloud_vtk_points)
        self.cloud_octree.SetDataSet(self.cloud_vtk_polydata)
        self.cloud_octree.BuildLocator()
        """汽车点云actor"""
        self.v_filter.SetInputData(self.cloud_vtk_polydata)
        self.v_filter.Update()
        self.mapper.SetInputData(self.v_filter.GetOutput())
        self.cloud_actor.SetMapper(self.mapper)
        self.cloud_actor.GetProperty().SetPointSize(3)
        self.cloud_point_data = self.v_filter.GetOutput().GetPointData()
        self.cloud_color.SetNumberOfComponents(3)
        self.cloud_color.SetName('Colors')

        self.build_car()

    def build_car(self):
        """汽车点云染色"""
        self.cloud_color.SetNumberOfTuples(len(self.np_cloud))
        for i in range(len(self.np_cloud)):
            self.cloud_color.SetTuple(i, (0, 140, 90))
        self.cloud_point_data.SetScalars(self.cloud_color)
        self.cloud_point_data.Modified()
        self.cloud_vtk_polydata.Modified()

    def color_points(self, cloud_clipped_points):
        """对被切割点染色"""
        for idx, dis in cloud_clipped_points.items():
            self.cloud_color.SetTuple(idx, (255, 0, 0))
        self.cloud_point_data.SetScalars(self.cloud_color)
        self.cloud_point_data.Modified()
        self.cloud_vtk_polydata.Modified()
        # print(len(self.cloud_clipped_points))
        # print(self.cloud_clipped_points)

def create_wind_planes(fov: float, hor: float, length: float, height1:float, height2:float) -> vtk.vtkPlanes:
    """返回一个张角为fov，侧张角为hov，射程为length的喷水模型"""
    fov = tan(radians(fov / 2))
    hor = tan(radians(hor / 2))
    ratio = height1/(height2+height1)

    thickness = hor* height1
    n = [[0, 1, fov],
         [0, -1, fov],
         [-1, 0, hor],
         [1, 0, hor],
         [0, 0, -1],
         [0, 0, 1]]

    p = [[0, length * ratio / 2, 0],
         [0, -length * ratio / 2, 0],
         [-thickness, 0, 0],
         [thickness, 0, 0],
         [0, 0, -height2],
         [0, 0, 0]]
    points = vtk.vtkPoints()
    points.SetData(numpy_to_vtk(p))
    planes = vtk.vtkPlanes()
    planes.SetNormals(numpy_to_vtk(n))
    planes.SetPoints(points)
    return planes

def create_vtk_planes(fov: float, length: float, thickness: float) -> vtk.vtkPlanes:
    """返回一个张角为fov，侧张角为hov，射程为length的喷水模型"""
    fov = tan(radians(fov / 2))
    hor = 30
    hor = tan(radians(hor / 2))
    n = [[0, 1, fov],
         [0, -1, fov],
         [-1, 0, hor],
         [1, 0, hor],
         [0, 0, -1],
         [0, 0, 1]]

    p = [[0, 0, 0],
         [0, 0, 0],
         [-thickness / 2, 0, 0],
         [thickness / 2, 0, 0],
         [0, 0, -length],
         [0, 0, 0]]
    points = vtk.vtkPoints()
    points.SetData(numpy_to_vtk(p))
    planes = vtk.vtkPlanes()
    planes.SetNormals(numpy_to_vtk(n))
    planes.SetPoints(points)
    return planes

def get_actor(p: vtk.vtkPlanes):
    actor = vtk.vtkActor()
    mapper = vtk.vtkPolyDataMapper()
    frustum = vtk.vtkFrustumSource()
    frustum.ShowLinesOff()
    frustum.SetPlanes(p)
    mapper.SetInputConnection(frustum.GetOutputPort())
    actor.GetProperty().SetOpacity(.5)
    actor.SetMapper(mapper)
    return actor

if __name__ == '__main__':
    p = create_wind_planes(fov= 90, hor=30, length=1, height1=0.2, height2=1)
    p2 = create_vtk_planes(fov= 90, length=1, thickness=0.2,)
    point  = vtk.vtkAxesActor()
    actor = get_actor(p)
    actor2 = get_actor(p2)




    coneRender = vtk.vtkRenderer()

    coneRender.AddActor(actor)
    coneRender.AddActor(point)
    coneRender.AddActor(actor2)

    renWin = vtk.vtkRenderWindow()
    renWin.AddRenderer(coneRender)
    renWin.SetSize(3000, 3000)
    renWin.SetWindowName('cone')

    iren = vtk.vtkRenderWindowInteractor()
    iren.SetRenderWindow(renWin)
    iren.SetInteractorStyle(vtk.vtkInteractorStyleMultiTouchCamera())
    iren.Initialize()
    iren.Start()