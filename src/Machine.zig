const std = @import("std");
const machine_config = @import("./machine_config.zig");
const Instructions = @import("./Instructions.zig");

const debugPrint = std.debug.print;
const time = std.time;

const InsCode = machine_config.InsCode;
const ByteWidth = machine_config.ByteWidth;
const MEMORY_SIZE = machine_config.MEMORY_SIZE;

const Self = @This();

cpu: Cpu,
memory: [*]u8,
instructions: Instructions,
time_started: std.time.Instant,

pub const Cpu = struct {
    ip: ByteWidth, // instruction pointer
    sp: ByteWidth, // stack pointer
    fp: ByteWidth, // frame pointer
    gr0: ByteWidth, // general purpose register0
    gr1: ByteWidth, // general purpose register1
    flag: ByteWidth,
};

pub var memory_buffer = [_]u8{0} ** MEMORY_SIZE;

pub fn init() Self {
    return .{
        .cpu = .{
            .ip = 0,
            .sp = MEMORY_SIZE - 1,
            .fp = MEMORY_SIZE - 1,
            .flag = 0,
            .gr0 = 0,
            .gr1 = 0,
        },
        .memory = @ptrCast(@constCast(&memory_buffer)),
        .instructions = undefined,
        .time_started = undefined,
    };
}

pub fn run(self: *Self, length: usize) void {
    const cpu: *Cpu = &self.cpu;
    const memory: [*]u8 = self.memory;

    self.time_started = time.Instant.now() catch unreachable;
    self.instructions = Instructions.init(self);
    const ins_tab = self.instructions.ins_tab;

    while (true) {
        const opcode: u8 = memory[cpu.ip] >> 2;
        // run a instruction;
        ins_tab[opcode](&self.instructions);
        if (cpu.ip >= @as(ByteWidth, @intCast(length))) break;
    }
}

pub fn printDebugInfo(self: *Self) void {
    const end = time.Instant.now() catch unreachable;
    const elapsed: f64 = @floatFromInt(end.since(self.time_started));
    const cpu: *Cpu = &self.cpu;
    const memory: [*]u8 = self.memory;

    debugPrint("{!}\n", .{self.cpu});
    debugPrint("memory{any}\n", .{
        memory[cpu.sp .. cpu.sp + 0x10],
    });
    debugPrint("Time elapsed is: {d:.3}ms\n", .{elapsed / time.ns_per_ms});
}

// flag:
// 0 => zero
// 1 => carry
