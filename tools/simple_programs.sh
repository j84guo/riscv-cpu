#!/bin/bash

OUTPUT_DIR=txt
mkdir -p $OUTPUT_DIR

for INPUT_FILE in simple-programs/*.x; do
    python3 replace_macro.py $(dirname $INPUT_FILE) $(basename $INPUT_FILE)
    iverilog -g2005 simple_programs_testbench.v
    BASENAME=$(basename $INPUT_FILE)
    OUTPUT_FILE=$OUTPUT_DIR/${BASENAME%.*}.txt
    echo "Writing: $OUTPUT_FILE"
    ./a.out > $OUTPUT_FILE
done
