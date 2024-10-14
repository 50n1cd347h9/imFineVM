#!/bin/sh


# execute binary
./zig-out/bin/imFineVM  ./src/executable/label.bin  2>&1 | awk 'NR == 1 { print $0 }'
