.PHONY: build run attach start_opencd start_jlink start_jlink_rtt restore_rom

#OPENOCD_INTERFACE ?= misc/openocd-jlink.cfg
OPENOCD_INTERFACE ?= misc/openocd-stlink.cfg

TARGET_ELF ?= target/thumbv7em-none-eabihf/release/app

build:
	@# We do build --release first, it shows compile error messages (objdump doesn't)
	cargo build --release
	cargo objdump -q --release -- -h | ./misc/rom_stats.py

run: build
	cargo run --release -q

attach:
	arm-none-eabi-gdb -q -x gdb/attach.gdb ${TARGET_ELF}

start_openocd:
	openocd -f ${OPENOCD_INTERFACE} -f target/stm32f1x.cfg

start_jlink:
	JLinkGDBServer -AutoConnect 1 -Device GD32F307VE -If SWD -Speed 4000 -nogui

start_jlink_rtt:
	JLinkRTTClient

start_probe_run_rtt:
	probe-run --chip STM32F107RC --no-flash ${TARGET_ELF}

misc/orig-firmware.elf: misc/orig-firmware.bin
	arm-none-eabi-objcopy -I binary -O elf32-little --rename-section .data=.text --change-address 0x08000000 $< $@

restore_rom: misc/orig-firmware.elf
	arm-none-eabi-gdb -q -x gdb/run.gdb $<
