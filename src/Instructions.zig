const std = @import("std");
const SubInstructions = @import("./SubInstructions.zig");
const Machine = @import("./Machine.zig");
const machine_config = @import("./machine_config.zig");

const print = std.debug.print;
const pow = std.math.pow;
const shr = std.math.shr;
const debugPrint = std.debug.print;

const write = SubInstructions.write;
const fetch = SubInstructions.fetch;
const deref = SubInstructions.deref;
const copy = SubInstructions.copy;

const Cpu: type = Machine.Cpu;

const Imm32: type = machine_config.Imm32;
const Imm16: type = machine_config.Imm16;
const Imm8: type = machine_config.Imm8;
const Ref: type = machine_config.Ref;
const Reg: type = machine_config.Reg;
const ByteWidth = machine_config.ByteWidth;
const SignedByteWidth = machine_config.SignedByteWidth;
const InsCode = machine_config.InsCode;
const Ext = machine_config.Ext;
const RegIdx: type = machine_config.RegIdx;

const EXT_MSK = machine_config.EXT_MSK;
const OPC_SZ = machine_config.OPC_SZ;
const MACHINE_BYTES: u8 = machine_config.MACHINE_BYTES;
const MEMORY_SIZE = machine_config.MEMORY_SIZE;

const Self = @This();

ip_ofs: u8,
cpu: *Cpu,
memory: [*]u8,
ext: u8,
len: u8,
first: u8,
imm_len: u8,
first_reg: Reg,
ins_tab: []*const fn (*Self) void,

pub var _self: *Self = undefined;
comptime {
    @export(&_self, .{ .name = "_self_instructions", .linkage = .strong });
}

pub fn registerSelf(self: *Self) void {
    _self = self;
}

pub fn init(machine: *Machine) Self {
    const cpu = &machine.cpu;
    const memory = machine.memory;

    return Self{
        .cpu = cpu,
        .memory = memory,
        .ins_tab = instruction_table().init(),
        .ip_ofs = undefined,
        .ext = undefined,
        .len = undefined,
        .first = undefined,
        .imm_len = undefined,
        .first_reg = undefined,
    };
}

inline fn getLen(self: *Self) u8 {
    return self.memory[self.cpu.ip + OPC_SZ] >> 5;
}

inline fn getExt(self: *Self) u8 {
    return self.memory[self.cpu.ip] & EXT_MSK;
}

inline fn getFirstOprand(self: *Self) u8 {
    return self.memory[self.cpu.ip + OPC_SZ] << 3 >> 5;
}

inline fn getImmLen(len: u8) u8 {
    return if (len != 0) @divExact(pow(u8, 2, len + 2), 8) else 0;
}

inline fn getReg(self: *Self, reg: u8) Reg {
    switch (reg) {
        @intFromEnum(RegIdx.ip) => return &self.cpu.ip,
        @intFromEnum(RegIdx.sp) => return &self.cpu.sp,
        @intFromEnum(RegIdx.fp) => return &self.cpu.fp,
        @intFromEnum(RegIdx.flag) => return &self.cpu.flag,
        @intFromEnum(RegIdx.gr0) => return &self.cpu.gr0,
        @intFromEnum(RegIdx.gr1) => return &self.cpu.gr1,
        else => return &self.cpu.gr0,
    }
}

pub fn refresh(self: *Self) void {
    const ext = self.getExt();
    const len = self.getLen();
    const first = self.getFirstOprand();
    const imm_len = getImmLen(len);
    const first_reg = self.getReg(first);

    self.ip_ofs = 2;
    self.ext = ext;
    self.len = len;
    self.first = first;
    self.imm_len = imm_len;
    self.first_reg = first_reg;
}

pub fn instruction_table() type {
    return struct {
        var func_vec: [@intFromEnum(InsCode.count)]*const fn (*Self) void = undefined;

        pub fn init() *[@intFromEnum(InsCode.count)]*const fn (*Self) void {
            func_vec[@intFromEnum(InsCode.push)] = push;
            func_vec[@intFromEnum(InsCode.pop)] = pop;
            func_vec[@intFromEnum(InsCode.add)] = add;
            func_vec[@intFromEnum(InsCode.sub)] = sub;
            func_vec[@intFromEnum(InsCode.div)] = div;
            func_vec[@intFromEnum(InsCode._and)] = _and;
            func_vec[@intFromEnum(InsCode._or)] = _or;
            func_vec[@intFromEnum(InsCode.xor)] = xor;
            func_vec[@intFromEnum(InsCode._shl)] = _shl;
            func_vec[@intFromEnum(InsCode.ldr)] = ldr;
            func_vec[@intFromEnum(InsCode.ldm)] = ldm;
            func_vec[@intFromEnum(InsCode.cmp)] = cmp;
            func_vec[@intFromEnum(InsCode.jmp)] = jmp;
            func_vec[@intFromEnum(InsCode.jz)] = jz;
            func_vec[@intFromEnum(InsCode.jg)] = jg;
            func_vec[@intFromEnum(InsCode.jl)] = jl;
            func_vec[@intFromEnum(InsCode.call)] = call;
            func_vec[@intFromEnum(InsCode.ret)] = ret;
            func_vec[@intFromEnum(InsCode.nop)] = nop;
            return &func_vec;
        }
    };
}

inline fn getSigned(unsigned: ByteWidth) SignedByteWidth {
    return @as(SignedByteWidth, @bitCast(unsigned));
}

inline fn getDst(rel: SignedByteWidth) ByteWidth {
    const origin: SignedByteWidth = @intCast(_self.cpu.ip + _self.ip_ofs + _self.imm_len);
    return @as(ByteWidth, @intCast(origin + rel));
}

fn jump_switching(self: *Self) ByteWidth {
    var dst_loc: ByteWidth = 0;
    switch (self.ext) {
        Ext.imm => {
            const dst_loc_rel = deref(self.cpu.ip + 2, MACHINE_BYTES);
            dst_loc = getDst(getSigned(dst_loc_rel));
        },
        else => undefined,
    }
    return dst_loc;
}

fn arithmetic_switching(self: *Self, execute: fn (Reg, ByteWidth) ByteWidth) ByteWidth {
    const memory: [*]u8 = self.memory;
    const ip: ByteWidth = self.cpu.ip;
    const imm_len: u8 = self.imm_len;
    const dst_reg: Reg = self.first_reg;
    self.ip_ofs += self.imm_len;
    switch (self.ext) {
        Ext.imm => {
            const src = fetch(ip + 2, imm_len);
            return execute(dst_reg, src);
        },
        Ext.reg => {
            const src_reg: Reg = self.getReg(memory[ip + 2]);
            return execute(dst_reg, src_reg.*);
        },
        Ext.imm_ref => {
            const src_ref: Ref = fetch(ip + 2, imm_len);
            const src = deref(src_ref, imm_len);
            return execute(dst_reg, src);
        },
        Ext.reg_ref => {
            const src_ref: Ref = (self.getReg(memory[ip + 2])).*;
            const src = deref(src_ref, MACHINE_BYTES);
            return execute(dst_reg, src);
        },
        else => unreachable,
    }
}

pub fn pop(self: *Self) void {
    self.ip_ofs += self.imm_len;
    defer {
        self.cpu.ip += self.ip_ofs;
        self.cpu.sp += MACHINE_BYTES;
        self.refresh();
    }
    switch (self.ext) {
        Ext.reg => {
            const dst_reg: Reg = self.getReg(self.memory[self.cpu.ip + 2]);
            const src = deref(self.cpu.sp, MACHINE_BYTES);
            dst_reg.* = src;
        },
        Ext.imm_ref => {
            const dst_ref = deref(self.cpu.ip + 2, self.imm_len);
            copy(dst_ref, self.cpu.sp, MACHINE_BYTES);
        },
        Ext.reg_ref => {
            const dst_ref = (self.getReg(self.memory[self.cpu.ip + 2])).*;
            copy(dst_ref, self.cpu.sp, MACHINE_BYTES);
        },
        else => {
            return;
        },
    }
}

pub fn push(self: *Self) void {
    self.ip_ofs += self.imm_len;
    defer {
        self.cpu.ip += self.ip_ofs;
        self.cpu.sp -= MACHINE_BYTES;
        self.refresh();
    }
    switch (self.ext) {
        Ext.imm => {
            copy(self.cpu.sp - MACHINE_BYTES, self.cpu.ip + 2, self.imm_len);
        },
        Ext.reg => {
            const src = (self.getReg(self.memory[self.cpu.ip + 2])).*;
            write(self.cpu.sp - MACHINE_BYTES, src, MACHINE_BYTES);
        },
        Ext.imm_ref => {
            const src_ref: Ref = fetch(self.cpu.ip + 2, MACHINE_BYTES);
            copy(self.cpu.sp - MACHINE_BYTES, src_ref, MACHINE_BYTES);
        },
        Ext.reg_ref => {
            const src_ref: Ref = (self.getReg(self.memory[self.cpu.ip + 2])).*;
            copy(self.cpu.sp - MACHINE_BYTES, src_ref, MACHINE_BYTES);
        },
        else => {
            return;
        },
    }
}

pub fn add(self: *Self) void {
    defer {
        self.cpu.ip += self.ip_ofs;
        self.refresh();
    }
    _ = self.arithmetic_switching(struct {
        pub const _self = self;
        fn execute(dst_reg: Reg, src: ByteWidth) ByteWidth {
            dst_reg.* += src;
            return 0;
        }
    }.execute);
}

pub fn sub(self: *Self) void {
    defer {
        self.cpu.ip += self.ip_ofs;
        self.refresh();
    }
    _ = self.arithmetic_switching(struct {
        fn execute(dst_reg: Reg, src: ByteWidth) ByteWidth {
            dst_reg.* -= src;
            return 0;
        }
    }.execute);
}

pub fn mul(self: *Self) void {
    defer {
        self.cpu.ip += self.ip_ofs;
        self.refresh();
    }
    _ = self.arithmetic_switching(struct {
        fn execute(dst_reg: Reg, src: ByteWidth) ByteWidth {
            dst_reg.* *= src;
            return 0;
        }
    }.execute);
}

pub fn div(self: *Self) void {
    defer {
        self.cpu.ip += self.ip_ofs;
        self.refresh();
    }
    _ = self.arithmetic_switching(struct {
        fn execute(dst_reg: Reg, src: ByteWidth) ByteWidth {
            dst_reg.* /= src;
            return 0;
        }
    }.execute);
}

pub fn _and(self: *Self) void {
    defer {
        self.cpu.ip += self.ip_ofs;
        self.refresh();
    }
    _ = self.arithmetic_switching(struct {
        fn execute(dst_reg: Reg, src: ByteWidth) ByteWidth {
            dst_reg.* &= src;
            return 0;
        }
    }.execute);
}

pub fn _or(self: *Self) void {
    defer {
        self.cpu.ip += self.ip_ofs;
        self.refresh();
    }
    _ = self.arithmetic_switching(struct {
        fn execute(dst_reg: Reg, src: ByteWidth) ByteWidth {
            dst_reg.* |= src;
            return 0;
        }
    }.execute);
}

pub fn xor(self: *Self) void {
    defer {
        self.cpu.ip += self.ip_ofs;
        self.refresh();
    }
    _ = self.arithmetic_switching(struct {
        fn execute(dst_reg: Reg, src: ByteWidth) ByteWidth {
            dst_reg.* ^= src;
            return 0;
        }
    }.execute);
}

pub fn _shl(self: *Self) void {
    defer {
        self.cpu.ip += self.ip_ofs;
        self.refresh();
    }
    _ = self.arithmetic_switching(struct {
        fn execute(dst_reg: Reg, src: ByteWidth) ByteWidth {
            dst_reg.* = dst_reg.* << @as(u5, @intCast(src));
            return 0;
        }
    }.execute);
}

pub fn cmp(self: *Self) void {
    var flag: ByteWidth = 0b00;
    defer {
        self.cpu.ip += self.ip_ofs;
        self.cpu.flag &= @as(u8, @intCast(flag));
        self.refresh();
    }
    flag = self.arithmetic_switching(struct {
        fn execute(dst_reg: Reg, src: ByteWidth) ByteWidth {
            const dst: ByteWidth = dst_reg.*;
            if (dst > src) {
                return 0b00;
            } else if (dst == src) {
                return 0b01;
            } else {
                return 0b10;
            }
        }
    }.execute);
}

// load to register
pub fn ldr(self: *Self) void {
    self.ip_ofs += self.imm_len;
    defer {
        self.cpu.ip += self.ip_ofs;
        self.refresh();
    }
    const memory: [*]u8 = self.memory;
    const ip: ByteWidth = self.cpu.ip;
    const imm_len: u8 = self.imm_len;
    const dst_reg: Reg = self.first_reg;
    switch (self.ext) {
        Ext.imm => {
            const src = fetch(ip + 2, imm_len);
            dst_reg.* = src;
        },
        Ext.reg => {
            const src_reg: Reg = self.getReg(memory[ip + 2]);
            dst_reg.* = src_reg.*;
        },
        Ext.imm_ref => {
            const src_ref: Ref = fetch(ip + 2, imm_len);
            const src = fetch(src_ref, MACHINE_BYTES);
            dst_reg.* = src;
        },
        Ext.reg_ref => {
            const src_ref: Ref = (self.getReg(memory[ip + 2])).*;
            const src = fetch(src_ref, @sizeOf(ByteWidth) / 8);
            dst_reg.* = src;
        },
        else => {
            return;
        },
    }
}

// load to memory pointed by register
pub fn ldm(self: *Self) void {
    self.ip_ofs += self.imm_len;
    defer {
        self.cpu.ip += self.ip_ofs;
        self.refresh();
    }
    const memory: [*]u8 = self.memory;
    const ip: ByteWidth = self.cpu.ip;
    const imm_len: u8 = self.imm_len;
    const dst_reg: Reg = self.first_reg;
    const dst_ref: Ref = dst_reg.*;
    switch (self.ext) {
        Ext.imm => {
            copy(dst_ref, ip + 2, imm_len);
        },
        Ext.reg => {
            const src_reg: Reg = self.getReg(memory[ip + 2]);
            write(dst_ref, src_reg.*, MACHINE_BYTES);
        },
        Ext.imm_ref => {
            const src_ref: Ref = fetch(ip + 2, imm_len);
            copy(dst_ref, src_ref, MACHINE_BYTES);
        },
        Ext.reg_ref => {
            const src_ref: Ref = (self.getReg(memory[ip + 2])).*;
            copy(dst_ref, src_ref, MACHINE_BYTES);
        },
        else => {
            return;
        },
    }
}

pub fn jmp(self: *Self) void {
    var dst_loc: ByteWidth = 0;
    defer {
        self.cpu.ip = dst_loc;
        self.refresh();
    }
    dst_loc = self.jump_switching();
}

pub fn jg(self: *Self) void {
    var dst_loc: ByteWidth = 0;
    defer {
        self.cpu.ip = dst_loc;
    }
    if ((self.cpu.flag & 0b00) == 0b00) {
        dst_loc = self.jump_switching();
    } else {
        dst_loc += self.cpu.ip + OPC_SZ;
    }
}

pub fn jz(self: *Self) void {
    var dst_loc: ByteWidth = 0;
    defer {
        self.cpu.ip = dst_loc;
        self.refresh();
    }
    if ((self.cpu.flag & 0b01) == 0b01) {
        dst_loc = self.jump_switching();
    } else {
        dst_loc += self.cpu.ip + OPC_SZ;
    }
}

pub fn jl(self: *Self) void {
    var dst_loc: ByteWidth = 0;
    defer {
        self.cpu.ip = dst_loc;
        self.refresh();
    }
    if ((self.cpu.flag & 0b10) == 0b10) {
        dst_loc = self.jump_switching();
    } else {
        dst_loc += self.cpu.ip + OPC_SZ;
    }
}

pub fn call(self: *Self) void {
    defer {
        self.cpu.sp -= MACHINE_BYTES;
        self.refresh();
    }
    const ret_addr = self.cpu.ip + self.ip_ofs + self.imm_len;
    write(self.cpu.sp - MACHINE_BYTES, ret_addr, MACHINE_BYTES);
    self.jmp();
}

pub fn ret(self: *Self) void {
    const ret_addr: ByteWidth = deref(self.cpu.sp, MACHINE_BYTES);
    defer {
        self.cpu.sp += MACHINE_BYTES;
        self.cpu.ip = ret_addr;
        self.refresh();
    }
}

pub fn nop(self: *Self) void {
    self.ip_ofs = OPC_SZ;
    defer {
        self.cpu.ip += self.ip_ofs;
        self.refresh();
    }
}
