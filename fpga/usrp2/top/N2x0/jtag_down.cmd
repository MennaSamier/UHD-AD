setMode -bs
setCable -port auto
identify -inferir
identifyMPM
assignFile -p 2 -file "build-ML605/u2plus.bit"
program -p 2
quit 
