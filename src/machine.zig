const std = @import("std");
const print = std.debug.print;
const machine_config = @import("./machine_config.zig");

const ByteWidth = machine_config.ByteWidth;
const MEMORY_SIZE = machine_config.MEMORY_SIZE;

pub const Machine = struct {
    cpu: Cpu,
    memory: [*]u8,
};

pub const Cpu = struct {
    ip: ByteWidth, // instruction pointer
    sp: ByteWidth, // stack pointer
    fp: ByteWidth, // frame pointer
    gr0: ByteWidth, // general purpose register
    gr1: ByteWidth, // general purpose register
    flag: ByteWidth,
};

var memory = &[_]u8{0} ** MEMORY_SIZE;

pub fn initMachine() Machine {
    return .{
        .cpu = .{
            .ip = 0,
            .sp = MEMORY_SIZE - 1,
            .fp = MEMORY_SIZE - 1,
            .flag = 0,
            .gr0 = 0,
            .gr1 = 0,
        },
        .memory = @ptrCast(@constCast(memory)),
    };
}

// flag:
// 0 => zero
// 1 => carry
