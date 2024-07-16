const instructions = @import("./instructions.zig");
const machine_ = @import("./machine.zig");
const machine_config = @import("./machine_config.zig");
const std = @import("std");
const time = std.time;
const process = std.process;
const Instant = time.Instant;
const Timer = time.Timer;
const print = std.debug.print;

const opr_sz = machine_config.opr_sz;
const ByteWidth = machine_config.ByteWidth;
const SignedByteWidth = machine_config.SignedByteWidth;
const Instruction = instructions.Instruction;
const initInstructions = instructions.initInstructions;
const MEMORY_SIZE = machine_config.MEMORY_SIZE;

const Cpu: type = machine_.Cpu;

pub fn main() !void {
    const args = try process.argsAlloc(std.heap.page_allocator);
    defer process.argsFree(std.heap.page_allocator, args);
    if (args.len < 2) {
        return;
    }

    const executable = try std.fs.cwd().openFile(
        args[1],
        .{ .mode = .read_only },
    );
    defer executable.close();
    var machine = machine_.initMachine();
    const cpu: *Cpu = &machine.cpu;
    const memory: [*]u8 = machine.memory;
    const length: usize = executable.readAll(memory[0..MEMORY_SIZE]) catch unreachable;

    var ins = Instruction().init(&machine);

    print("{!}\n", .{machine});

    const start = try Instant.now();
    while (true) {
        const opcode: u8 = memory[cpu.ip] >> 2;
        ins.instruction[opcode](&ins);
        if (cpu.ip >= @as(ByteWidth, @intCast(length))) break;
    }
    const end = try Instant.now();

    print("{!}\n", .{machine});
    print("memory\n{p} = {any}\n", .{ &memory[cpu.sp], memory[cpu.sp .. cpu.sp + 0x10] });
    const elapsed: f64 = @floatFromInt(end.since(start));
    print("Time elapsed is: {d:.3}ms\n", .{elapsed / time.ns_per_ms});
}
