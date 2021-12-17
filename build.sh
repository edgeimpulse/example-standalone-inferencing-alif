set -e
SCRIPTPATH="$( cd "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"

echo "Building standalone classifier"

cd $SCRIPTPATH

rm -rf build
mkdir build
cd build
cmake ..
make -j

echo "Building standalone classifier OK"