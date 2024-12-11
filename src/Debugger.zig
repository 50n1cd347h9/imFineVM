const std = @import("std");
const re = @import("./regex.zig");
const machine_config = @import("./machine_config.zig");

const stdout = std.io.getStdOut().writer();
const stdin = std.io.getStdIn().reader();
const mem = std.mem;
const math = std.math;
const fmt = std.fmt;

const OPC_SZ = machine_config.OPC_SZ;
const MEMORY_SIZE = machine_config.MEMORY_SIZE;

const InsCode = machine_config.InsCode;
const ByteWidth = machine_config.ByteWidth;
const Cpu = @import("./Machine.zig").Cpu;
const Self = @This();

const Machine = struct {
    cpu: *Cpu,
    memory: [*]u8,
};

machine: Machine,
addr_words_map: std.AutoHashMap(ByteWidth, []const u8), // (addr, word)
last_inspected: ByteWidth, // address last inspected
word_origins: std.ArrayList(ByteWidth),

var gpa = std.heap.GeneralPurposeAllocator(.{}){};
const a = gpa.allocator();

pub fn init(cpu: *Cpu, memory: [*]u8) Self {
    return Self{
        .machine = Machine{ .cpu = cpu, .memory = memory },
        .addr_words_map = std.AutoHashMap(ByteWidth, []const u8).init(a),
        .last_inspected = 0,
        .word_origins = std.ArrayList(ByteWidth).init(a),
    };
}

pub fn deinit(self: *Self) void {
    var itr = self.addr_words_map.iterator();
    while (itr.next()) |entry|
        a.free(entry.value_ptr.*);
    self.addr_words_map.deinit();
    self.word_origins.deinit();
}

inline fn info(msg: []const u8) void {
    stdout.print("[*] {s}\n", .{msg}) catch {};
}

inline fn warn(msg: []const u8) void {
    stdout.print("[!] {s}\n", .{msg}) catch {};
}

pub fn debug(self: *Self) void {
    if (self.word_origins.items.len == 0)
        self.traceWords();

    self.deleteBrkPt(null);

    printCpuState(&self.machine);
    self.repl();
}

fn printCpuState(machine: *Machine) void {
    const cpu = machine.cpu;
    stdout.print(
        \\  ip: 0x{x}
        \\  sp: 0x{x}
        \\  fp: 0x{x}
        \\ gr0: 0x{x}
        \\ gr1: 0x{x}
        \\
        \\
    , .{
        cpu.ip,
        cpu.sp,
        cpu.fp,
        cpu.gr0,
        cpu.gr1,
    }) catch {};
}

fn printStack(memory: [*]u8) void {
    _ = memory;
}

/// read, evaluate, print loop
fn repl(self: *Self) void {
    var buf = [_]u8{0} ** 0x100;

    while (true) {
        stdout.print("debug> ", .{}) catch {};
        const line = readLine(&buf);
        self.eval(line);
    }
}

inline fn readLine(buf: *[0x100]u8) []u8 {
    const err_msg = "death";

    @memset(buf, 0);
    return stdin.readUntilDelimiter(buf, '\n') catch {
        mem.copyForwards(u8, buf, err_msg);
        return buf[0..err_msg.len];
    };
}

var start: usize = 0;
fn eval(self: *Self, line: []const u8) void {
    start = 0;

    while (start < line.len) {
        const cmd = readOne(line);
        self.command(cmd, line);
    }
}

fn readOne(line: []const u8) []const u8 {
    var end: usize = start;

    if (start >= line.len)
        return line[0..0];

    for (line[start..line.len]) |char| {
        if (char == ' ' or char == '\t') {
            start += 1;
            end = start;
        } else {
            break;
        }
    }

    for (line[start..line.len]) |char| {
        if (char == ' ' or char == '\t')
            break;
        end += 1;
    }

    defer start = end + 1;
    return line[start..end];
}

fn command(self: *Self, cmd_str: []const u8, line: []const u8) void {
    switch (cmd_str[0]) {
        'i' => self.printBrkPts(),
        'd' => self.deleteBrkPt(line),
        's' => self.step(),
        'r' => run(),
        'b' => self._break(line),
        else => {
            warn("unknown command: ");
            warn(cmd_str);
        },
    }
}

/// insert 'int 0' into next instruction and run
fn step(self: *Self) void {
    const curr_ip = self.machine.cpu.ip;
    const len = self.getLen(curr_ip);
    const next_ip = 2 + getImmLength(len);

    self.submitBrkPt(next_ip);
    run();
}

fn run() void {}

/// break at specified address.
fn _break(self: *Self, line: []const u8) void {
    const arg_str = readOne(line);

    if (toInt(arg_str)) |addr| {
        if (!self.isValidAddr(addr))
            return;
        self.submitBrkPt(addr);
    } else {
        warn("invalid argument");
        info(arg_str);
        info("usage: break <addr>");
    }
}

fn submitBrkPt(self: *Self, addr: ByteWidth) void {
    if (self.addr_words_map.contains(addr)) // if given address already submitted
        return;

    const word = self.getNPushWord(addr) catch "\x00";
    self.addr_words_map.put(addr, word) catch {};

    self.insertInt0(addr);
}

inline fn insertInt0(self: *Self, addr: ByteWidth) void {
    const int_0 = [_]u8{ // debug instruction
        @intFromEnum(InsCode.int) << 2,
        1 << 5,
        0,
    };
    // insert int_0
    for (int_0, 0..) |byte, i|
        self.machine.memory[addr + i] = byte;
}

fn getNPushWord(self: *Self, addr: ByteWidth) ![]u8 {
    const tmp = self.getLen(addr);
    const len = 2 + getImmLength(tmp);
    return try a.dupe(u8, self.machine.memory[addr..len]);
}

fn deleteBrkPt(self: *Self, line: ?[]const u8) void {
    var addr = self.machine.cpu.ip;

    if (line != null) {
        const addr_str = readOne(line.?);
        if (toInt(addr_str)) |_addr| {
            addr = _addr;
        } else {
            warn("invalid argument:");
            info(addr_str);
        }
    }

    if (self.addr_words_map.get(addr)) |word| {
        for (word, 0..) |byte, i|
            self.machine.memory[addr + i] = byte;
    }
}

fn printBrkPts(self: *Self) void {
    _ = self;
}

/// check if given address is aligned
fn isValidAddr(self: *Self, addr: ByteWidth) bool {
    for (self.word_origins.items) |origin|
        if (addr == origin)
            return true;
    return false;
}

// TODO: check if opcode is valid
fn traceWords(self: *Self) void {
    info("Inspecting memory, please wait...");

    self.word_origins.append(0) catch
        warn("internal error: arraylist");

    while (self.last_inspected < MEMORY_SIZE) {
        const tmp = self.getLen(self.last_inspected);
        self.last_inspected += (2 + getImmLength(tmp));

        self.word_origins.append(self.last_inspected) catch
            warn("internal error: arraylist");
    }

    info("done\n");
}

inline fn getLen(self: *Self, addr: ByteWidth) u8 {
    return self.machine.memory[addr + OPC_SZ] >> 5;
}

inline fn getImmLength(len: u8) u8 {
    return if (len != 0) @divExact(math.pow(u8, 2, len + 2), 8) else 0;
}

fn toInt(buf: []const u8) ?ByteWidth {
    if (buf.len > 2 and buf[0] == '0' and buf[1] == 'x') // if hexadecimal
        return fmt.parseInt(ByteWidth, buf[2..buf.len], 16) catch null;
    return fmt.parseInt(ByteWidth, buf, 10) catch null;
}
