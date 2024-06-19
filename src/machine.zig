const std = @import("std");
const print = std.debug.print;
const machine_config = @import("./machine_config.zig");

const ByteWidth: type = machine_config.ByteWidth;
const MEMORY_SIZE = machine_config.MEMORY_SIZE;

pub const Machine = struct {
    cpu: Cpu,
    memory: [*]u8,
};

pub const Cpu = struct {
    ip: ByteWidth,
    sp: ByteWidth,
    flag: u8,
};

var memory = &[_]u8{0} ** MEMORY_SIZE;

pub fn initMachine() Machine {
    return .{
        .cpu = .{
            .ip = 0,
            .sp = MEMORY_SIZE - 1,
            .flag = 0,
        },
        .memory = @ptrCast(@constCast(memory)),
    };
}
