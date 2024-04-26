git pull
make CONF=Debug__RPI_ clean
rm -r ./dist/Debug__RPI_
make CONF=Debug__RPI_ -j4  -Wall
cd ./dist/Debug__RPI_/GNU-Linux/
sudo ./superviseur-robot
cd ../../../
