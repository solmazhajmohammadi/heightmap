#chmod +x test.sh
#!/bin/bash

echo "--START     " `date` >&2

cmake .
make -j20 
#./main -i input.ply -o outputname

echo "--END       " `date` >&2
