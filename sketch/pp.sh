iconv -c -f utf-8 -t ISO-8859-1 PCLapCounterHW.ino | a2ps --pro=color -C -1 -M letter -f 9 -g --pretty-print='C++' -o - --stdin=PCLapCounterHW --encoding=ISO-8859-1 | ps2pdf - PCLapCounterHW.pdf
