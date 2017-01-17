# rtt-component-template
This is an empty template for a RTT component.

### Usage:
```bash
cd to_your_workspace
git clone https://github.com/cogimon/rtt-component-template.git
cd rtt-component-template
mkdir build
cd build
cmake -DCMAKE_PREFIX_PATH=/vol/cogimon/cogimon-minimal-nightly ..
make
```
this should compile and create a package/library like:
```bash
orocos
└── gnulinux
    └── RttExamples
        └── libRttExamples-gnulinux.so
```
Within the build directory:
```bash
export RTT_COMPONENT_PATH=$RTT_COMPONENT_PATH:$(pwd)/orocos
```
happy coding!
