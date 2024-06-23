const std = @import("std");
const shr = std.math.shr;
const print = std.debug.print;
const instructions = @import("./instructions.zig");
const machine_ = @import("./machine.zig");
const machine_config = @import("./machine_config.zig");

const opr_sz = machine_config.opr_sz;
const ByteWidth = machine_config.ByteWidth;
const SignedByteWidth = machine_config.SignedByteWidth;
const MEMORY_SIZE = machine_config.MEMORY_SIZE;

const Cpu: type = machine_.Cpu;
const opc_msk: u8 = 0b11111000;

pub fn main() !void {
    var machine = machine_.initMachine();
    const instruction = instructions.initInstructions();

    const cpu: *Cpu = &machine.cpu;
    const memory: [*]u8 = machine.memory;

    const executable = try std.fs.cwd().openFile(
        "./src/exe.bin",
        .{ .mode = .read_only },
    );
    defer executable.close();
    const length: usize = executable.readAll(memory[0..MEMORY_SIZE]) catch unreachable;

    while (true) {
        const opcode: u8 = shr(u8, memory[cpu.ip], 3);

        print("opcode = {x}\n", .{opcode});
        instruction[opcode](&machine);
        print("cpu = {!}\n\n", .{cpu});

        if (cpu.ip >= @as(ByteWidth, @intCast(length))) break;
    }
}
