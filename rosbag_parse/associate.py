## in another script we have find the association of color and depth and now
## in this script we will pick the correspondents of color and depth images
## according their timestamps then rename them according the standards of
##  bundlefusion dataset and then store them in separate folders.
 
## step one: read the assoc.txt into this project.
 
import sys
import os
import shutil

 
def read_file_list(filename):
    """
    Reads a trajectory from a text file.
    File format:
    The file format is "stamp d1 d2 d3 ...", where stamp denotes the time stamp (to be matched)
    and "d1 d2 d3.." is arbitary data (e.g., a 3D position and 3D orientation) associated to this timestamp.
    Input:
    filename -- File name
    Output:
    dict -- dictionary of (stamp,data) tuples
    """
    file = open(filename)
    data = file.read()
    lines = data.replace(",", " ").replace("\t", " ").split("\n")
    #strip() function can be used to remove space located in the head or the tail places of a string
    # v.strip('0')remove 0
 
    # list = [[v.strip() for v in line.split(" ") if v.strip() != ""] for line in lines if
    #         len(line) > 0 and line[0] != "#"]
    # list = [(float(l[0]), l[1:]) for l in list if len(l) > 1]
 
    list= []
    listResult = []
    for line in lines:
        tmpList = []
        if len(line) > 0 and line[0] != "#":
            for v in line.split(" "):
                if v.strip() != "":
                    tmpList.append(v.strip())
        list.append(tmpList)
 
    for l in list:
        if len(l) > 1:
            listResult.append((float(l[0]), l[1:]))
 
    return dict(listResult)


def associate(first_list, second_list, offset, max_difference):
    """
    Associate two dictionaries of (stamp,data). As the time stamps never match exactly, we aim
    to find the closest match for every input tuple.
    Input:
    first_list -- first dictionary of (stamp,data) tuples
    second_list -- second dictionary of (stamp,data) tuples
    offset -- time offset between both dictionaries (e.g., to model the delay between the sensors)
    max_difference -- search radius for candidate generation
    Output:
    matches -- list of matched tuples ((stamp1,data1),(stamp2,data2))
    """
    ## obatin all keys
    first_keys = list(first_list)
    second_keys = list(second_list)
    potential_matches = [(abs(a - (b + offset)), a, b)
                         for a in first_keys
                         for b in second_keys
                         if abs(a - (b + offset)) < max_difference]
    potential_matches.sort()
    matches = []
    for diff, a, b in potential_matches:
        if a in first_keys and b in second_keys:
            first_keys.remove(a)
            second_keys.remove(b)
            matches.append((a, b))
 
    matches.sort()
    return matches
 
 
def read_data_from_file(file_path):
 
    associ_file = open(file_path)
    data = associ_file.read()
    lines = data.replace(',', ' ').replace('\t', ' ').split('\n')
 
    print("read {} from {}.".format(len(lines), file_path))
    # come here we have obtain every line of this associ.txt and in each line we can index arbitrary part of it
    # we mainly work on the first and the third part and then to find the correspondent color and depth image,
    # and the rename them and store them.
    list = [[v.strip() for v in line.split(" ") if v.strip() != " "] for line in lines if len(line) > 0 and line[0] != "#"]
    return list


def load_images_rename_store(file_data, input_folder, output_folder):
    print("The length of file is {}.".format(len(file_data)))
    print("the path of image_folder is {}.".format(input_folder))
    print("the path of output_folder is {}.".format(output_folder))
 
    input_rgb_path = os.path.join(input_folder, "rgb")
    input_depth_path = os.path.join(input_folder, "depth")
    # if not os.path.exists(rgb_path):
    #     os.mkdir(rgb_path)
    # if not os.path.exists(depth_path):
    #     os.mkdir(depth_path)
    rgbs = os.listdir(input_rgb_path)
    depths = os.listdir(input_depth_path)
 
    couples = []
    print("............................")
    for data in file_data:
        couple_list = []
        rgb_name = data[1].split("/")[1]
        depth_name = data[3].split("/")[1]
 
        couple_list.append(rgb_name)
        couple_list.append(depth_name)
        couples.append(couple_list)
 
    print("length of couples is {}.".format(len(couples)))

    output_rgb_path = os.path.join(output_folder, "rgb")
    output_depth_path = os.path.join(output_folder, "depth")
    if not os.path.exists(output_rgb_path):
        os.mkdir(output_rgb_path)
    if not os.path.exists(output_depth_path):
        os.mkdir(output_depth_path)

    frame_id = 0
    for couple in sorted(couples):
        if couple[0] in sorted(rgbs) and couple[1] in sorted(depths):
            print("\nframe_id is {}.\n".format(frame_id))
 
            print(os.path.join(output_rgb_path, couple[0]))
            print(os.path.join(output_folder, "frame-%06d.color.png" % frame_id))
            print("\n")
            print(os.path.join(input_depth_path, couple[1]))
            print(os.path.join(output_depth_path, "frame-%06d.depth.png" % frame_id))
            # print(output_folder + "/depth/" + "frame-" + str(frame_id).zfill(6) + ".depth.png")

            shutil.copyfile(os.path.join(input_rgb_path, couple[0]),
                            os.path.join(output_rgb_path, "frame-%06d.color.png" % frame_id))
            shutil.copyfile(os.path.join(input_depth_path, couple[1]),
                            os.path.join(output_depth_path, "frame-%06d.depth.png" % frame_id))
            frame_id += 1


if __name__ == '__main__':

    DATA_ROOT = "/home/hongkai/disk/datasets/803/k4a_data/20200707/change2/02"
    rgb_txt = os.path.join(DATA_ROOT, "rgb.txt")
    depth_txt = os.path.join(DATA_ROOT, "depth.txt")
    offset = 0.012
    max_diff = 0.1

    input_path = os.path.join(DATA_ROOT, "raw")
    output_path = os.path.join(DATA_ROOT, "matched")
    associate_txt = os.path.join(DATA_ROOT, "associate.txt")

    rgb_list = read_file_list(rgb_txt)
    depth_list = read_file_list(depth_txt)
    matches = associate(rgb_list, depth_list, offset, max_diff)

    with open(associate_txt, 'w') as fp:
        for a, b in matches:
            fp.write("%f %s %f %s" % (a, " ".join(rgb_list[a]), b - float(offset), " ".join(depth_list[b])) + '\n')

    list = read_data_from_file(associate_txt)
    load_images_rename_store(list, input_path, output_path)