import os
names = {}
rootdir = 'TeamCode/src/main/java/org/firstinspires/ftc/teamcode'
for path, subdirs, files in os.walk(rootdir):
	for name in files:
		if '.java' in name:
			with open(os.path.join(path, name)) as f:
				try:
					names[name] = sum(1 for line in f if line.strip())
				except UnicodeDecodeError:
					print('EXCEPTION: ' + name)

for name, lines in names.items():
    print(f'{name}: {lines} lines')

print('-------------------------------------------------')
print(f'Total lines of code (excluding blank lines): {sum(names.values())}')
