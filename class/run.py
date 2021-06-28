import os,numpy
from chemical import ChemicalBuilder

if __name__ == '__main__':

    folder_path = "2dcars/"
    car_id = os.listdir(folder_path)
    for i in car_id:
        # if i not in  ["2dcar_21.npy"]:
        #     continue
        if i.endswith("npy"):
            print(i)
            path2d = "2dcars/" + i
            with open(path2d, 'rb') as f_pos:
                chem_builder = ChemicalBuilder(list(numpy.load(f_pos)))
                chem_builder.Build()