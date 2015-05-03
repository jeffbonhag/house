sox %1 -r 8000 -c 1 -e unsigned-integer -b 8 %1.raw
iojs drink.js %1.raw %2