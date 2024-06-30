const instructions = @import("./instructions.zig");
const machine_ = @import("./machine.zig");
const machine_config = @import("./machine_config.zig");
const std = @import("std");
const time = std.time;
const Instant = time.Instant;
const Timer = time.Timer;
const print = std.debug.print;

const opr_sz = machine_config.opr_sz;
const ByteWidth = machine_config.ByteWidth;
const SignedByteWidth = machine_config.SignedByteWidth;
const MEMORY_SIZE = machine_config.MEMORY_SIZE;

const Cpu: type = machine_.Cpu;

pub fn main() !void {
    var machine = machine_.initMachine();
    const instruction = instructions.initInstructions();

    const cpu: *Cpu = &machine.cpu;
    const memory: [*]u8 = machine.memory;

    const executable = try std.fs.cwd().openFile(
        "./src/executable/test.bin",
        .{ .mode = .read_only },
    );
    defer executable.close();
    const length: usize = executable.readAll(memory[0..MEMORY_SIZE]) catch unreachable;
    //_ = length;
    print("{!}\n\n", .{machine});

    var prev_ip: ByteWidth = 0;
    const start = try Instant.now();
    while (true) {
        const opcode: u8 = memory[cpu.ip] >> 2;
        prev_ip = cpu.ip;
        instruction[opcode](&machine);
        //if (cpu.ip == prev_ip) break;
        if (cpu.ip >= @as(ByteWidth, @intCast(length))) break;
    }
    const end = try Instant.now();

    print("{!}\n", .{machine});
    print("memory\n{p} = {any}\n", .{ &memory[cpu.sp], memory[cpu.sp .. cpu.sp + 0x10] });

    const elapsed: f64 = @floatFromInt(end.since(start));
    print("Time elapsed is: {d:.3}ms\n", .{elapsed / time.ns_per_ms});
}
