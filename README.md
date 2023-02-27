# Edge Impulse POC new drivers

Currently using forked alif repos as submodules to match their build system identically at first, and track deviations 
from the public release in terms of support for appkit. So update submodules:
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

```

What works:
- EI SDK
- UART/VCOM
- LCD via lvgl

What doesn't work:
- Camera (on david's board)

What hasn't been tested:
- Microphone
