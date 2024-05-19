The goal of this project is to tune a V990 rotax V-twin engine in real-road conditions, but the methodology applies to any motorcycle.
This repository keeps track of all files needed for the logging and fuel-table improvements.

Requirements on the motorcycle
- Power command installed (for fuel-table improvements)
- TPS sensor has been properly calibrated into the power commander
- Need a way to mount a bosch lsu 4.9 lambda sensor. I have installed a lambda sensor bung (M18 x 1.5) on each downpipe
- Way to turn on/off the electronics. I have wired the power on the lighting system, which allows turning the system on/off on demand

Requirements on the person using this project
- Understanding how AFR works and what the targets are
- Understanding how lambda sensor works, how to change controller settings and when to turn it on/off
- OK with simple software projects, electronics and 3D printing

Materials you need
- Innovate LC-2 wideband lambda controller. Other kits can be used but make sure to take not of the output voltage range vs. AFR and status outputs
- USB-to-Serial converter for configuring the LC-2
- Power commander PCIII or PCV
- Arduino
- 3D printer
- brass inserts and screws of various sizes
- connectors, crimp contacts and pliers
- soldering materials
- experimentation PCB
- resistors (ideally ones with 1% or better accuracy for automotive use that withstand typical TVS events)
- caps (ceramic is preferred)
- transistors
- ICs (555 timer and some other stuff)

Software I used
- LTspice
- fusion 360
- Logworks 3 with LM programmer
- arduino IDE
- Python environment
