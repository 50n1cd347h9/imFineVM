# Install
```shell
git clone https://github.com/50n1cd347h9/imFineVM.git
cd imFineVM
zig build
```
The built binary is `zig-out/bin/imFineVM`, but set the alias if necessary

# What is imFineVM
imFineVM is a virtual machine consisting of original instructions, cpu (and registers) and memory.

The instruction consists of at least 3 bytes.
Its structure is as follows
```
| opcode 6bits           | ext 2bits   | 
| len 2bits | reg 2bits| padding 3bits |
| immediate or register 1~128bits      |
```
## oprands
reg means register and indicates the first operand.
The third byte means immediate or register and indicates the second operand.

## registers
Each of the six registers is assigned the following numbers
|ip|sp|fp|flag|gro|gr1|
|-|-|-|-|-|-|
|000|001|010|011|100|101|

## opcode
|push|pop|add|sub|mul|div|and|or|xor|shl|ldr|ldm|cmp|jmp|jg|jz|jl|call|ret|nop|
|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|
|0x1|0x2|0x3|0x4|0x5|0x6|0x7|0x8|0x9|0xa|0xb|0xc|0xd|0xe|0xf|0x10|0x11|0x12|0x13|0x14|

## ext
## len
len means length of second oprand
|0 bit|8|16|32|64|128|
|-|-|-|-|-|-|
|000|001|010|011|100|101|

The implemented instructions are as follows
