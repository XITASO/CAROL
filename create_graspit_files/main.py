import subprocess
import sys
"""
sys.argv[1] := path to ivcon executable
sys.argv[2] := path to stl file to be converted to graspit modell
sys.argv[3] := graspit home directory
"""


def create_iv_file(filename):
    subprocess.run([sys.argv[1], sys.argv[2], sys.argv[3] + "/models/objects/" + filename + ".iv"])


def get_filename_from_path(absolute_path):
    tmp_list = str.split(absolute_path, "/")
    return tmp_list[len(tmp_list) - 1]


def get_object_name_from_filename(filename):
    tmp_list = str.split(filename, ".stl")
    if(len(tmp_list) == 1):
        tmp_list.clear()
        tmp_list = str.split(filename, ".STL")
    return tmp_list[0]


def create_graspit_object_xml(name):
    file = open(sys.argv[3] + "/models/objects/template.xml", "rt")
    object_file = open(sys.argv[3] + "/models/objects/" + name + ".xml", "wt")
    for line in file:
        object_file.write(line.replace("template", name))
    file.close()
    object_file.close()


def create_world_file_for_object(name):
    file = open(sys.argv[3] + "/worlds/template_world.xml", "rt")
    world_file = open(sys.argv[3] + "/worlds/youbot_" + name + ".xml", "wt")
    for line in file:
        world_file.write(line.replace("template", name))
    file.close()
    world_file.close()


if __name__ == '__main__':
    filename = get_filename_from_path(sys.argv[2])
    object_name = get_object_name_from_filename(filename)
    create_iv_file(object_name)
    create_graspit_object_xml(object_name)
    create_world_file_for_object(object_name)
