import os

src_path = 'Src'
file_lists = []
for root, dirs, files in os.walk(src_path):
    for file in files:
        if '.cpp' in file:
            file_lists.append(os.path.join(root, file))

for i in range(len(file_lists)-1):
    ss = file_lists[i] + " \\"
    print(ss)
print(file_lists[-1])
