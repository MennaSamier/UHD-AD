setMode -bs

setCable -p auto

identify

identifyMPM

attachflash -position 2 -bpi "XCF128X"

assignfiletoattachedflash -position 2 -file "build-ML605/u2plus.mcs"

Program -p 2 -dataWidth 16 -rs1 NONE -rs0 NONE -bpionly -e -loadfpga 

quit 
