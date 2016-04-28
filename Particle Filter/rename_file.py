import os

os.chdir('saved_img/collection-1')
file_list = os.listdir()
print(*file_list, sep='\n')
for file in file_list:
    no = file.split('-')[0]
    print(no)
    if len(no) == 2:
        os.renames(file, '0' + file)