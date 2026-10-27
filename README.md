# Lomsourt OS 🚀

A custom 64-bit bare-metal operating system built from scratch in C and Assembly.

## 🎥 OS Boot & Execution Demo
Watch the real-time boot process, hardware initialization, and core system execution in the QEMU emulator:

[![Lomsourt OS Demo](https://img.youtube.com/vi/jFwFWF6j2es/hqdefault.jpg)](https://youtu.be/jFwFWF6j2es)
*(Click the image above to watch the video)*

## 🛠️ Core Features & Modules
* Custom Bootloader & Initialization: Switches to 64-bit Long Mode and initializes core CPU features.
* ATA / IDE Hard Disk Driver: Supports 28-bit LBA PIO mode for reading/writing storage sectors.
* Intel e1000 Network Driver: Initial configuration for Gigabit Ethernet networking.
* Memory & Privilege Management: Custom GDT, TSS, and Paging setup (Ring 0 / Ring 3 transitions).

## 💻 Tech Stack & Environment
* Architecture: x86_64
* Emulator: QEMU (using SeaBIOS)
* Languages: C & x86 Assembly
