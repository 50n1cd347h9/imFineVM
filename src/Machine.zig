const std = @import("std");
const machine_config = @import("./machine_config.zig");
const Instructions = @import("./Instructions.zig");
const VideoController = @import("./VideoController.zig");
const Debugger = @import("./Debugger.zig");

const debugPrint = std.debug.print;
const time = std.time;
const Thread = std.Thread;

const InsCode = machine_config.InsCode;
const ByteWidth = machine_config.ByteWidth;
const MEMORY_SIZE = machine_config.MEMORY_SIZE;

const Self = @This();

cpu: Cpu,
memory: [*]u8,
video_controller: VideoController,
instructions: Instructions,
debugger: Debugger,
time_started: std.time.Instant,

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
        .video_controller = VideoController.init(),
        .debugger = undefined,
        .time_started = undefined,
    };
}

pub fn deinit(self: *Self) void {
    _ = self;
}

pub fn run(self: *Self, length: usize) void {
    self.time_started = time.Instant.now() catch unreachable;

    self.debugger = Debugger.init(&self.cpu, self.memory);
    defer self.debugger.deinit();

    self.instructions = Instructions.init(self);
    self.instructions.registerSelf();
    self.instructions.refresh();

    const machine_thread = machineThread(self, length);
    const video_thread = videoThread(self);
    defer {
        machine_thread.join();
        video_thread.join();
    }
}

inline fn machineThread(self: *Self, length: usize) Thread {
    return Thread.spawn(
        .{},
        Self.run_cpu,
        .{ self, length },
    ) catch unreachable;
}

inline fn videoThread(self: *Self) Thread {
    return Thread.spawn(
        .{},
        VideoController.run,
        .{&self.video_controller},
    ) catch unreachable;
}

pub fn run_cpu(self: *Self, length: usize) void {
    const cpu: *Cpu = &self.cpu;

    while (cpu.ip < @as(ByteWidth, @intCast(length))) {
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
    _ = opcode;
    // self.instructions.ins_tab[opcode](&self.instructions);
    self.instructions.int();
}

pub fn printDebugInfo(self: *Self) void {
    const end = time.Instant.now() catch unreachable;
    const elapsed: f64 = @floatFromInt(end.since(self.time_started));
    const cpu: *Cpu = &self.cpu;
    const memory: [*]u8 = self.memory;

    debugPrint(
        \\{!}
        \\memory{any}
        \\{d:.3} ms
        \\
    , .{
        self.cpu,
        memory[cpu.sp .. cpu.sp + 0x10],
        elapsed / time.ns_per_ms,
    });
}
