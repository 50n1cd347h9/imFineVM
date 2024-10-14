const Instructions = @import("./Instructions.zig");
const machine_config = @import("./machine_config.zig");
const Machine = @import("Machine.zig");
const std = @import("std");

const io = std.io;
const stdout = io.getStdOut().writer();
const process = std.process;

const opr_sz = machine_config.opr_sz;
const ByteWidth = machine_config.ByteWidth;
const SignedByteWidth = machine_config.SignedByteWidth;
const MEMORY_SIZE = machine_config.MEMORY_SIZE;

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

    var machine = Machine.init();
    const length: usize = try executable.readAll(machine.memory[0..MEMORY_SIZE]);

    machine.run(length);
    machine.printDebugInfo();
}

test "run label.bin" {
    const test_string =
        "Machine.Cpu{ .ip = 36, .sp = 65535, .fp = 65535, .gr0 = 63, .gr1 = 0, .flag = 0 }\n";

    const allocator = std.testing.allocator;
    var stdout_buf = std.ArrayList(u8).init(allocator);
    defer stdout_buf.deinit();
    var err_buf = std.ArrayList(u8).init(allocator);
    defer err_buf.deinit();

    const runvm = [_][]const u8{"./src/test.sh"};
    var child = std.process.Child.init(&runvm, allocator);
    child.stderr_behavior = .Pipe;
    child.stdout_behavior = .Pipe;
    try child.spawn();
    try child.collectOutput(&stdout_buf, &err_buf, 0x1000);
    _ = try child.wait();

    try std.testing.expect(std.mem.eql(
        u8,
        stdout_buf.items,
        test_string,
    ));
}
