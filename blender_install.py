import argparse
import os

# The script used to install package for the 
# blender internal python. Only works for blender 2.80
# Only one arguement about the package
parser = argparse.ArgumentParser()
parser.add_argument('package', type=str)
args = parser.parse_args()


def main():
    # Check the blender path in environment variable
    if not 'BLENDER_PATH' in os.environ:
        print('Please setup and BLENDER_PATH environment variable.')
        exit(1)

    # OK
    blender_path = os.environ['BLENDER_PATH']
    assert os.path.exists(blender_path)
    blender_folder = os.path.dirname(blender_path)
    
    # The python binary
    blender_pip_bin = os.path.join(blender_folder, '2.80/python/bin/pip')
    command = blender_pip_bin + ' install %s' % args.package
    print(command)
    os.system(command)


if __name__ == '__main__':
    main()
