const std = @import("std");
const print = std.debug.print;
const machine_config = @import("./machine_config.zig");
const instructions = @import("./instructions.zig");
const machine_ = @import("./machine.zig");

const ByteWidth: type = machine_config.ByteWidth;
const Cpu: type = machine_.Cpu;
const MEMORY_SIZE = machine_config.MEMORY_SIZE;
const opcode_msk: u8 = 0b00111111;

pub fn main() !void {
    var machine = machine_.initMachine();
    const instruction = instructions.initInstructions();

    const cpu: *Cpu = &machine.cpu;
    const memory: [*]u8 = machine.memory;

    print("{!}\n", .{machine});
    print("{!}\n", .{cpu.sp});

    while (true) {
        const opcode: u8 = memory[cpu.ip] & opcode_msk;
        _ = opcode;
        //instruction[opcode](&machine);
        instruction[0](&machine);
        break;
    }

    print("{!}\n", .{machine});
    print("{!}\n", .{cpu.sp});
}
