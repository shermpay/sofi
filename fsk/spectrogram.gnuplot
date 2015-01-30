filename="/tmp/fft.txt"

reset
set palette positive
set pm3d map clip4in

set xlabel "Time (s)"

set ylabel "Frequency (Hz)"
set logscale y

set cblabel "dBFS"
set cbrange [-100:0]

set autoscale fix
splot filename using 1:2:3 notitle with pm3d

# vim:ft=gnuplot
