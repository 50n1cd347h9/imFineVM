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
### oprands
reg means register and indicates the first operand.
The third byte means immediate or register and indicates the second operand.

The implemented instructions are as follows
