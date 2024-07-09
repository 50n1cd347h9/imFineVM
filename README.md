# Install
```shell
git clone https://github.com/50n1cd347h9/imFineVM.git
cd imFineVM
zig build
```
The built binary is `zig-out/bin/imFineVM`, but set the alias if necessary

# What is imFineVM
imFineVM is a virtual machine consisting of original instructions, cpu (and registers) and memory.<br>
By default, registers are `32 bits` wide and the stack is aligned by register size.

The instruction consists of at least 3 bytes.
Its structure is as follows
```
| opcode 6bits           | ext 2bits   | 
| len 2bits | reg 2bits| padding 3bits |
| immediate or register 1~128bits      |
```
## oprands
reg means register and indicates the first operand.<br>
The third byte means immediate or register and indicates the second operand.<br>
For instructions that take only one operand, that operand is represented as a second operand

## registers
Each of the six registers is assigned the following numbers
|ip|sp|fp|flag|gr0|gr1|
|-|-|-|-|-|-|
|000|001|010|011|100|101|

`ip` is an instruction pointer<br>
`sp` is a stack pointer<br>
`fp` is a frame pointer<br>
`flag` indicates the state of the virtual machine<br>
`gr0` and `gr1` are general-purpose registers


## opcode
|push|pop|add|sub|mul|div|and|or|xor|shl|ldr|ldm|cmp|jmp|jg|jz|jl|call|ret|nop|
|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|-|
|0x1|0x2|0x3|0x4|0x5|0x6|0x7|0x8|0x9|0xa|0xb|0xc|0xd|0xe|0xf|0x10|0x11|0x12|0x13|0x14|

`ldr` and `ldm` mean load to register and load to memory, respectively.
In assembly, both are denoted as `ld`.

## ext
ext reveals what the second operand indicates
|imm|reg|[imm]|[reg]|
|-|-|-|-|
|00|01|10|11|

[] means that inside it is a reference to memory
## len
len means length of second oprand
|0 bit|8|16|32|64|128|
|-|-|-|-|-|-|
|000|001|010|011|100|101|

## flag register
|0|1|
|-|-|
|zero|carry|
