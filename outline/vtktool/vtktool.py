import random, math
import numpy as np
from vtkmodules.util.numpy_support import numpy_to_vtk
from vtkmodules import all as vtk
from typing import Iterable


def color_actor(actor, color):
    if color is True:
        actor.GetProperty().SetColor(random.random(), random.random(), random.random())
    elif isinstance(color, Iterable):
        actor.GetProperty().SetColor(*color)


def point_actor(points_list, color=False, point_size=3):
    if type(points_list) == vtk.vtkPoints:
        points = points_list
    else:
        if type(points_list) == list:
            points_list = np.array(points_list)
        points = vtk.vtkPoints()
        points.SetData(numpy_to_vtk(points_list))
    polydata = vtk.vtkPolyData()
    polydata.SetPoints(points)
    mapper = vtk.vtkPointGaussianMapper()
    mapper.SetInputData(polydata)
    mapper.EmissiveOff()
    mapper.SetScaleFactor(0.0)
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    color_actor(actor, color)
    actor.GetProperty().SetPointSize(point_size)
    return actor


def poly_actor(polydata, v_filter=False):
    mapper = vtk.vtkPolyDataMapper()
    actor = vtk.vtkActor()

    if v_filter:
        v_filter = vtk.vtkVertexGlyphFilter()
        v_filter.SetInputData(polydata)
        mapper.SetInputConnection(v_filter.GetOutputPort())
    else:
        mapper.SetInputData(polydata)
    actor.SetMapper(mapper)
    return actor


def stl_actor(stl_path_or_reader):
    if type(stl_path_or_reader) is str:
        reader = vtk.vtkSTLReader()
        reader.SetFileName(stl_path_or_reader)
    elif stl_path_or_reader.IsA('vtkSTLReader'):
        reader = stl_path_or_reader
    else:
        print('类型错误')
        return
    reader.Update()
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(reader.GetOutputPort())
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    return actor


def line_actor(points_list, color=False, line_width=8):
    if type(points_list) == list:
        points_list = np.array(points_list)
    points = vtk.vtkPoints()
    points.SetData(numpy_to_vtk(points_list))
    poly_line = vtk.vtkPolyLine()
    poly_line.GetPointIds().SetNumberOfIds(points.GetNumberOfPoints())
    for i in range(points.GetNumberOfPoints()):
        poly_line.GetPointIds().SetId(i, i)
    grid = vtk.vtkUnstructuredGrid()
    grid.Allocate(1, 1)
    grid.InsertNextCell(poly_line.GetCellType(), poly_line.GetPointIds())
    grid.SetPoints(points)
    mapper = vtk.vtkDataSetMapper()
    mapper.SetInputData(grid)
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    color_actor(actor, color)
    actor.GetProperty().SetLineWidth(line_width)
    return actor


def source_actor(source):
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputConnection(source.GetOutputPort())
    actor = vtk.vtkActor()
    actor.SetMapper(mapper)
    return actor


def vtk_show(*args, color=False, interactor=None, style=None):
    render = vtk.vtkRenderer()
    render.SetBackground(0, 0, 0)
    # cam = render.GetActiveCamera()
    # cam.SetPosition([3.41, 10.25, 0])
    # cam.SetFocalPoint([3.41, 0, 0])
    # cam.SetViewUp([0, 0, -1])
    # Renderer Window
    window = vtk.vtkRenderWindow()
    window.AddRenderer(render)
    window.SetPosition(0, 3000)
    window.SetSize(3000, 1600)
    # System Event
    if not interactor:
        interactor = vtk.vtkRenderWindowInteractor()
    interactor.SetRenderWindow(window)
    # Style
    if style:
        interactor.SetInteractorStyle(style)
    else:
        interactor.SetInteractorStyle(vtk.vtkInteractorStyleMultiTouchCamera())
    # Insert Actor
    for item in args:
        if type(item) == list or type(item) == np.ndarray:
            actor = point_actor(item, color)
        elif item.IsA('vtkProp3D') or item.IsA('vtkImageActor') or item.IsA('vtkAssembly'):
            actor = item
        elif issubclass(type(item), vtk.vtkPolyDataAlgorithm):
            actor = source_actor(item)
        else:
            print('error')
            continue
        render.AddActor(actor)
    interactor.Initialize()
    interactor.Start()


def MinMax3D(source):
    p_min = source[0].copy()
    p_max = source[0].copy()
    for item in source:
        for i in range(3):
            if item[i] > p_max[i]:
                p_max[i] = item[i]
            elif item[i] < p_min[i]:
                p_min[i] = item[i]
    return p_min, p_max


def center_3D(source):
    if type(source) == list:
        source = np.array(source)
    center = source.mean(axis=0)
    return center


def pass_filter(source, axis: str, min=None, max=None, both=False):
    if axis in 'xX0':
        axis = 0
    elif axis in 'yY1':
        axis = 1
    elif axis in 'zZ2':
        axis = 2
    else:
        return source
    if min is None and max is None:
        return source
    elif min is None:
        res_in = [p for p in source if p[axis] <= max]
        res_out = [p for p in source if p[axis] > max]
    elif max is None:
        res_in = [p for p in source if p[axis] >= min]
        res_out = [p for p in source if p[axis] < min]
    else:
        res_in = [p for p in source if max >= p[axis] >= min]
        res_out = [p for p in source if p[axis] < min or p[axis] > max]
    if both:
        return res_in, res_out
    else:
        return res_in


def Up(cloud, distance, axis='y'):
    dict_axis = {
        'x': 0,
        'y': 1,
        'z': 2,
    }
    if axis not in dict_axis:
        return
    for i in cloud:
        i[dict_axis[axis]] += distance


def numpy_cube(size):
    l = []
    for i in range(size):
        for j in range(size):
            for k in range(size):
                l.append([i / size - .5, j / size - .5, k / size - .5])
    return np.array(l)