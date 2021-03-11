import os
from pathlib import Path
import sys

def create_folder(object_name):
    directory_path = str()
    user = os.getenv("USER")
    directory_path = "/home/" + str(user) + "/.gazebo/models/"
    directory_path += object_name
    Path(directory_path).mkdir(parents=True, exist_ok=True)


def get_filename_from_path(absolute_path):
    tmp_list = str.split(absolute_path, "/")
    return tmp_list[len(tmp_list) - 1]


def get_object_name_from_filename(filename):
    tmp_list = str.split(filename, ".stl")
    if(len(tmp_list) == 1):
        tmp_list.clear()
        tmp_list = str.split(filename, ".STL")
    return tmp_list[0]


def create_gazebo_files(file_path):
    path = get_filename_from_path(file_path)
    object_name = get_object_name_from_filename(path)
    create_folder(object_name)
    user = os.getenv("USER")

    # write sdf file
    sdf_filename = "/home/" + user + "/.gazebo/models/" + object_name + "/model.sdf"
    sdf_file = open(sdf_filename, "wt")
    world_file = open("/home/" + user + "/.gazebo/models/template/model.sdf", "rt")
    for line in world_file:
        sdf_file.write(line.replace("template", object_name))
    sdf_file.close()
    world_file.close()

    # write config file
    sdf_filename = "/home/" + user + "/.gazebo/models/" + object_name + "/model.config"
    sdf_file = open(sdf_filename, "wt")
    world_file = open("/home/" + user + "/.gazebo/models/template/model.config", "rt")
    for line in world_file:
        sdf_file.write(line.replace("template", object_name))
    sdf_file.close()
    world_file.close()

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    create_gazebo_files(sys.argv[1])
