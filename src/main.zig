const std = @import("std");
const shr = std.math.shr;
const print = std.debug.print;
const instructions = @import("./instructions.zig");
const machine_ = @import("./machine.zig");
const machine_config = @import("./machine_config.zig");

const opr_sz = machine_config.opr_sz;
const ByteWidth = machine_config.ByteWidth;
const MEMORY_SIZE = machine_config.MEMORY_SIZE;

const Cpu: type = machine_.Cpu;
const opc_msk: u8 = 0b00111111;
//const opc_msk: u8 = 0b11111000;

pub fn main() !void {
    var machine = machine_.initMachine();
    const instruction = instructions.initInstructions();

    const cpu: *Cpu = &machine.cpu;
    const memory: [*]u8 = machine.memory;

    print("{!}\n", .{machine});

    while (true) {
        const opcode: u8 = shr(u8, memory[cpu.ip], 3);
        _ = opcode;
        //instruction[opcode](&machine);
        instruction[1](&machine);
        break;
    }

    print("{!}\n", .{machine});
    print("{!}\n", .{cpu.sp});
}
