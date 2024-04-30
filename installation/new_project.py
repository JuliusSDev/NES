import argparse
import os
import zipfile
import json


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
    #parser.add_argument('new_name', type=str, help='New name for the extracted folder')


    args = parser.parse_args()
    path = "../src/uebung_" + args.uebung

    extract_and_rename_zip("template.zip", path, 'uebung_'+args.uebung)
    print("[DEBUG] uebung_", args.uebung, "extracted")

    path_to_new_folder = path + '/uebung_' + args.uebung + '/project/'

    os.rename(path_to_new_folder + 'template.ioc', path_to_new_folder +'uebung_' + args.uebung + '.ioc')
    print(f"[DEBUG] Renamed .ioc file to {path_to_new_folder}uebung_{args.uebung}.ioc")

    vscode_folder = path_to_new_folder + ".vscode/launch.json"

    # Lesen der JSON-Datei und Entfernen der Kommentare
    with open(vscode_folder, 'r') as file:
        print("[DEBUG] Opening json-file")
        file_content = file.read()

    modified_content = file_content.replace("templatenew.elf", "uebung_" + args.uebung + ".elf")


    with open(vscode_folder, 'w') as file:
        file.write(modified_content)
        print(f"[DEBUG] Modified in {vscode_folder}")
            
    #     lines = file.readlines()
    #     json_content = ''.join([line.split('//')[0] for line in lines])

    # # Laden des JSON-Inhalts
    # data = json.loads(json_content)

    # # Zugriff und Änderung des "executable"-Attributs
    neuer_executable_wert = "./build/uebung_"+args.uebung +".elf"
    # data['configurations'][0]['executable'] = neuer_executable_wert

    # # Speichern der geänderten JSON-Datei
    # with open(vscode_folder, 'w') as file:
    #     json.dump(data, file, indent=4)

