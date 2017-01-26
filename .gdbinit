set target-async on
set confirm off
set history save
set mem inaccessible-by-default off
target extended-remote /dev/ttyACM0
mon swdp_scan
att 1
load ./build/stm32-DS18x20.elf
file ./build/stm32-DS18x20.elf
start
