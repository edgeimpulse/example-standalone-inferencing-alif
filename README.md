# Edge Impulse POC new drivers

Currently pointing to alif repo as submodules to match their build system identically, tracking deviations 
from the public release in terms of support for appkit. To build, update submodules:
```
git submodule update --init --recursive
```

build with docker:
```
docker build -t alif-firmware .
docker run --rm -it -v "${PWD}":/app alif-firmware /bin/bash -c "sh build.sh"

sudo cp build/bin/sectors/mram.bin /path/to/your/alif-seutils-dir/build/images/
sudo cp ei-app-cfg.json /path/to/your/alif-seutils-dir/build/config
```

then use SEUTIL tools to program:
```
cd /path/to/your/alif-seutils-dir
sudo truncate --size %16 build/images/mram.bin

./app-gen-toc -f build/config/ei-app-gcc.json
./app-write-mram
```

What works:
- EI SDK
- UART/VCOM
- LCD via lvgl
- Microphone

What doesn't work so far:
- Camera
