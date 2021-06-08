#!/bin/bash
# Basic while loop

set -eux -o pipefail

for i in {5526..5556}
do
	# /home/maggiewang/Workspace/blender_server/build/blender-2.82-linux64/blender \
	#     ./data/blender_template/blank.blend -b -d --python-use-system-env --python ./blender_server.py ${i} &	

	$BLENDER_PATH \
	    ./data/blender_template/blank.blend -b -d --python-use-system-env --python ./blender_server.py ${i} &	
done
echo Done

# counter=1030
# while [ $counter -le 1030 ]
# do
# # echo $counter
# ($BLENDER_PATH \
#     ./data/blender_template/blank.blend -b -d --python-use-system-env --python ./blender_server.py $counter
# (counter++)) &
# done
# echo All done

# #!/bin/bash
# set -eux -o pipefail

# $BLENDER_PATH \
#     ./data/blender_template/blank.blend -b -d --python-use-system-env --python ./blender_server.py ${1:-5556}
