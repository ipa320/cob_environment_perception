set bars 2.0
set style fill empty
set terminal pdf

set out "dist1.pdf"
plot "data_FSHD2.csv" using 1:4 title "FSHD" with lines, \
 "data_ESF2.csv" using 1:4 title "ESF" with lines, \
 "data_FPFH2.csv" using 1:4 title "FPFH" with lines, \
 "data_SHOT2.csv" using 1:4 title "SHOT" with lines


plot 'data_FSHD.csv' using 1:3:3:5:5:xticlabels(8) with candlesticks title 'FSHD' whiskerbars, \
    ''         using 1:4:4:4:4 with candlesticks lt -1 notitle, \
    ''         using 1:4 with linespoints lt 3 pt 13 notitle

plot 'data_ESF.csv' using 1:3:3:5:5:xticlabels(8) with candlesticks title 'ESF' whiskerbars, \
    ''         using 1:4:4:4:4 with candlesticks lt -1 notitle, \
    ''         using 1:4 with linespoints lt 3 pt 13 notitle

plot 'data_FPFH.csv' using 1:3:3:5:5:xticlabels(8) with candlesticks title 'FPFH' whiskerbars, \
    ''         using 1:4:4:4:4 with candlesticks lt -1 notitle, \
    ''         using 1:4 with linespoints lt 3 pt 13 notitle

plot 'data_SHOT.csv' using 1:3:3:5:5:xticlabels(8) with candlesticks title 'SHOT' whiskerbars, \
    ''         using 1:4:4:4:4 with candlesticks lt -1 notitle, \
    ''         using 1:4 with linespoints lt 3 pt 13 notitle


plot 'data_FSHD.csv' using 1:3:2:6:5:xticlabels(8) with candlesticks title 'FSHD' whiskerbars, \
    ''         using 1:4:4:4:4 with candlesticks lt -1 notitle, \
    ''         using 1:4 with linespoints lt 3 pt 13 notitle

plot 'data_ESF.csv' using 1:3:2:6:5:xticlabels(8) with candlesticks title 'ESF' whiskerbars, \
    ''         using 1:4:4:4:4 with candlesticks lt -1 notitle, \
    ''         using 1:4 with linespoints lt 3 pt 13 notitle

plot 'data_FPFH.csv' using 1:3:2:6:5:xticlabels(8) with candlesticks title 'FPFH' whiskerbars, \
    ''         using 1:4:4:4:4 with candlesticks lt -1 notitle, \
    ''         using 1:4 with linespoints lt 3 pt 13 notitle

plot 'data_SHOT.csv' using 1:3:2:6:5:xticlabels(8) with candlesticks title 'SHOT' whiskerbars, \
    ''         using 1:4:4:4:4 with candlesticks lt -1 notitle, \
    ''         using 1:4 with linespoints lt 3 pt 13 notitle
