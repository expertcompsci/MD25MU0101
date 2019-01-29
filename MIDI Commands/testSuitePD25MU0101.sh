#!/bin/bash
# testSuitePD25MU0101.sh
# Invoke this script with the MIDI hardware device port to
# use 
#       amdi -l 
# to list all available hardware device ports

E_BADARGS=85

if [ ! -n "$1" ]
then
  echo "Usage : testSuitePD25MU0101.sh <device>"
  echo "where <device> is one of the following available devices:"
  amidi -l 
  exit $E_BADARGS
fi  
ResultFileName=testRun$(date -d "today" +"%Y%m%d%H%M%S").txt
echo "Psalte PD25MU0101.sh Test Suite Results"  >> $ResultFileName
echo Date: $(date -d "today" +"%Y %m %d %H:%M:%S")  >> $ResultFileName
function runAndDump() 
{
    echo Processing: $2
    echo "$2 Result: " >> $ResultFileName
    amidi -p $1 -s $2
    amidi -p $1 -s DumpStatus.bin 
    amidi -p $1 -t 3 -d >> $ResultFileName  # The device gets 2 seconds to respond
    echo "" >> $ResultFileName
    echo "..................................." >> $ResultFileName
}
# Test Suite
runAndDump $1 ResetAll.bin
runAndDump $1 SetCable15.bin
runAndDump $1 SetCable16.bin
runAndDump $1 SetCableZero.bin
runAndDump $1 SetChannel17.bin
runAndDump $1 SetChannelMax.bin
runAndDump $1 SetChannelMin.bin
runAndDump $1 SetChannelZero.bin
runAndDump $1 SetSensitivity25.bin
runAndDump $1 SetSensitivity26.bin
runAndDump $1 SetSensitivityZero.bin
runAndDump $1 SetTranspose102.bin
runAndDump $1 SetTranspose103.bin
runAndDump $1 SetTransposeZero.bin
runAndDump $1 SetVelocityFalse.bin
runAndDump $1 SetVelocityTrue.bin
runAndDump $1 ResetAll.bin
runAndDump $1 DumpStatus.bin