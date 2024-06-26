#!/usr/bin/env python3

def repeat_content(input_file, output_file, repetitions=2000):
    with open(input_file, 'r', encoding='utf-8') as file:
        content = file.read()

    repeated_content = content * repetitions

    with open(output_file, 'w', encoding='utf-8') as file:
        file.write(repeated_content)

input_file = 'exe.bin'   
output_file = 'exe_.bin'

repeat_content(input_file, output_file)

