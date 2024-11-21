const std = @import("std");
const machine_config = @import("./machine_config.zig");
const Instructions = @import("./Instructions.zig");
const VideoController = @import("./VideoController.zig");

const debugPrint = std.debug.print;
const time = std.time;
const Thread = std.Thread;

const InsCode = machine_config.InsCode;
const ByteWidth = machine_config.ByteWidth;
const MEMORY_SIZE = machine_config.MEMORY_SIZE;

const Self = @This();

cpu: Cpu,
memory: [*]u8,
instructions: Instructions,
time_started: std.time.Instant,
video_controller: VideoController,

/// ip: instruction pointer
/// sp: stack pointer
/// fp: frame pointer
/// gr0: general purpose register0
/// gr1: general purpose register1
// flag:
// 0 => zero
// 1 => carry
pub const Cpu = struct {
    ip: ByteWidth,
    sp: ByteWidth,
    fp: ByteWidth,
    gr0: ByteWidth,
    gr1: ByteWidth,
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
        .video_controller = VideoController.init(),
    };
}

pub fn run(self: *Self, length: usize) void {
    self.time_started = time.Instant.now() catch unreachable;
    self.instructions = Instructions.init(self);
    self.instructions.refresh();

    const main_thread = Thread.spawn(
        .{},
        Self.run_cpu,
        .{ self, length },
    ) catch unreachable;
    const video_thread = Thread.spawn(
        .{},
        VideoController.run,
        .{&self.video_controller},
    ) catch unreachable;

    defer {
        main_thread.join();
        video_thread.join();
    }
}

pub fn run_cpu(self: *Self, length: usize) void {
    const cpu: *Cpu = &self.cpu;

    var i: usize = 0;
    while (cpu.ip < @as(ByteWidth, @intCast(length))) : (i += 1) {
        defer self.video_controller.state.terminate = true;
        self.step();
        // std.time.sleep(0);
    }
}

/// get opcode
/// in this context, opcode means index in ins_tab
inline fn getOpcode(self: *Self) u8 {
    return self.memory[self.cpu.ip] >> 2;
}

/// execute single insturction
inline fn step(self: *Self) void {
    const opcode = self.getOpcode();
    self.instructions.ins_tab[opcode](&self.instructions);
}

pub fn printDebugInfo(self: *Self) void {
    const end = time.Instant.now() catch unreachable;
    const elapsed: f64 = @floatFromInt(end.since(self.time_started));
    const cpu: *Cpu = &self.cpu;
    const memory: [*]u8 = self.memory;

    debugPrint(
        \\{!}
        \\memory{any}
        \\Time elapsed is: {d:.3}ms
        \\
    , .{
        self.cpu,
        memory[cpu.sp .. cpu.sp + 0x10],
        elapsed / time.ns_per_ms,
    });
}
