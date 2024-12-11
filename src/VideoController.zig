const std = @import("std");

const io = std.io;
const stdout = io.getStdOut().writer();
const debugPrint = std.debug.print;

const Self = @This();

state: CtrlrState,
video_memory: []u8,

const MEMORY_SIZE = 0x2000;

pub var memory_buffer = [_]u8{0} ** MEMORY_SIZE;

const CtrlrState = struct {
    wait: bool,
    terminate: bool,
};

pub fn init() Self {
    return .{
        .state = .{
            .wait = true,
            .terminate = false,
        },
        .video_memory = @ptrCast(@constCast(&memory_buffer)),
    };
}

pub fn run(self: *Self) void {
    std.mem.copyForwards(u8, self.video_memory, "ahiahi\n");

    while (!self.state.terminate) {
        if (self.state.wait)
            continue;

        self.printMemory();
        std.time.sleep(0);
    }

    debugPrint("exiting\n", .{});
}

fn printMemory(self: *Self) void {
    stdout.print("{s}", .{self.video_memory}) catch unreachable;
}
