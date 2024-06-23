import argparse
import os
import zipfile
import json
import subprocess
import re

# usage to create uebung2_1: python3 new_project.py 2_1 
#
# Meine Folderstruktur innerhalb von den projects aus dem tutorials
# 
# + projects
# |
# +- install
# |   +- new_project.py
# |
# +- src
#     +- uebung_1 .........
#     +- uebung_2_1 ......... und der Rest wird dann automatisch angelegt bei mir



STM32CUBEMX_DIR =  "STM32CubeMX/STM32CubeMX"
RELATIVE_PATH_FOR_NEW_PROJECT = "../src/"



def start_code_generator(iocfile):
    global STM32CUBEMX_DIR
    home_directory = os.path.expanduser("~")
    # Construct the full path to the program
    program_path = os.path.join(home_directory, STM32CUBEMX_DIR)

    subprocess.run([program_path,iocfile])

def extract_and_rename_zip(zip_file, target_folder, new_name):
    with zipfile.ZipFile(zip_file, 'r') as zip_ref:
        zip_ref.extractall(target_folder)
    
    # Get the extracted folder name
    extracted_folder = os.path.join(target_folder, os.path.splitext(os.path.basename(zip_file))[0])
    
    # Rename the extracted folder
    os.rename(extracted_folder, os.path.join(target_folder, new_name))

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Extract and rename a zip file.')
    
    parser.add_argument('uebung', type=str, help='Welche Übung bist du?')
    parser.add_argument('cubemx', type=str, nargs='?', default="--y", help='Start CubeMX? Default: yes (--y), no (--n)')
    args = parser.parse_args()
    
   
    path = RELATIVE_PATH_FOR_NEW_PROJECT + "uebung_" + args.uebung

    # extract and rename the project
    extract_and_rename_zip("template.zip", path, 'uebung_'+args.uebung)
    print("[DEBUG] uebung_", args.uebung, "extracted")

    path_to_new_folder = path + '/uebung_' + args.uebung + '/project/'

    # rename ioc file
    ioc_file =  path_to_new_folder +'uebung_' + args.uebung + '.ioc'
    os.rename(path_to_new_folder + 'template.ioc',ioc_file)
    print(f"[DEBUG] Renamed .ioc file to {ioc_file}")

    jsonPath = path_to_new_folder + ".vscode/launch.json"

    # Lesen der JSON-Datei und Entfernen der Kommentare
    with open(jsonPath, 'r') as file:
        print("[DEBUG] Opening json-file")
        file_content = file.read()

    modified_content = file_content.replace("templatenew.elf", "uebung_" + args.uebung + ".elf")


    with open(jsonPath, 'w') as file:
        file.write(modified_content)
        print(f"[DEBUG] Modified in {jsonPath}")

    if(args.cubemx != "--n"):
        current_directory = os.getcwd()
        current_directory = re.sub(r'installation', '', current_directory)
        current_directory +='src/uebung_' + args.uebung +'/uebung_' + args.uebung+'/project/'+'/uebung_' + args.uebung + '.ioc'
        print("[DEBUG] open {current_directory}")
        start_code_generator(current_directory)

    
