set target-async on
set confirm off
set mem inaccessible-by-default off

target extended-remote /dev/cu.usbmodemC3E687D1
monitor swdp_scan
attach 1
file build/BBB.elf


