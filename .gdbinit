set target-async on
set confirm off
set history save
set mem inaccessible-by-default off
target extended-remote /dev/cu.usbmodemBED9AFE1
mon swdp_scan
att 1
load ./build/stm32-DS18x20.elf
file ./build/stm32-DS18x20.elf
start
