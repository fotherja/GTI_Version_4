# GTI_Version_4
This repo contains the hardware and software files for GTIv4. This is being superseded by GTIv5 (https://github.com/fotherja/Grid_Tie_Inverter_V5)

Hardware:
KiCad was used to design the 4-Layer PCB. This was manufactured by <a href="https://www.pcbway.com/">PCBWay</a> (They produce good quality PCBs for competitive prices with excellent turnaround times).

The board uses gallium nitride MASTERGAN2s and an isolated driver chip. Four AMC1306 isolated current shunt modulators read the various analogue signals and an STM32L475 powers the board.    

Problems: 1) The EMC filter capacitors are incorrectly connected up. It may reduce the performance of the EMI filtering. 2) On testing, at about 300 Watts one of the MASTERGAN2s exploded. The exact reason is unclear and so this hardware has been redesigned into V5.

Software:
Software was developed in STM32CubeIDE. We got as far as implementing a PR controller which was extremely working well. Needs lots more work though.

Problems: I did not manage to get the DFSDM analogue watchdog feature working to provide protection in case of overcurrent or overvoltage. I asked the question on StackOverflow but got no answers (https://stackoverflow.com/questions/75287992/cant-get-the-analogue-watchdog-to-trigger-an-interrupt-on-the-dfsdm-peripheral)
