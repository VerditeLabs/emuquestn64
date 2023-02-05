#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <memory.h>


#define _op ((opcode >> 26) & 0x3f)
#define _special (opcode & 0x3f)
#define _regimm ((opcode >> 16) & 0x1f)

#define dbg(...)
#define die(...)
#define die_if(cond, ...)

#define disasm(fmt, ...)

typedef unsigned char u8;
typedef signed char s8;
typedef unsigned short u16;
typedef signed short s16;
typedef unsigned int u32;
typedef signed int s32;
typedef unsigned long long u64;
typedef signed long long s64;
typedef signed __int128 s128;
typedef unsigned __int128 u128;
typedef float f32;
typedef double f64;

int main() {
    printf("Hello, World!\n");
    return 0;
}

union reg {
    u8 ub[16];
    s8 sb[16];
    u16 uh[8];
    s16 sh[8];
    u32 uw[4];
    s32 sw[4];
    u64 ud[2];
    s64 sd[2];
    u128 uq[1];
    s128 sq[1];
    f32 f[4];
    f64 d[2];

};

struct n64 {
    union reg gpr[32];
    union reg fpr[32];
    union reg pc, hi, lo;
    bool indelay;
    u64 nextpc;
    bool llbit;
    u32 fcr0, fcr31;
    union reg cop0[32];
    u8 *rdram;
    u8* pifrom;
    u8* pifram;
    u8* cart;
};

struct n64* n64(char* pifpath, char* cartpath) {
    struct n64* n64 = malloc(sizeof(struct n64));
    n64->rdram = malloc(1024*1024*8);
    n64->pifram = malloc(64);
    n64->pifrom = malloc(2048);


    FILE* fp = fopen(pifpath, "rb");
    die_if(fp == NULL, "can't open pifpath %s", pifpath);
    fseek(fp, 0, SEEK_END);
    size_t sz = ftell(fp);
    die_if(sz != 2048, "pif wrong size");
    fseek(fp, 0, SEEK_SET);
    fread(n64->pifrom,1,2048,fp);
    fclose(fp);

    fp = fopen(cartpath, "rb");
    die_if(fp == NULL, "can't open cartpath %s",cartpath);
    fseek(fp, 0, SEEK_END);
    sz = ftell(fp);
    fseek(fp, 0, SEEK_SET);
    n64->cart = malloc(sz);
    fread(n64->cart,1,sz,fp);
    fclose(fp);

    n64->pc.uw[0] = 0x1fc00000;

    return n64;
}

#define CASE4(start, end, dbgstr, op) case start ... end: dbg(dbgstr); op; break;
#define CASE3(start, dbg, op) case start: dbg(dbgstr); op;

#define rwgen(T, name) \
T name(struct n64 *n64, u64 addr, T data, bool write) {\
    switch (addr) {    \
        CASE4(0x0, 0x003fffff, "rdram", ret = n64->rdram[addr & 0x3fffff];); \
        CASE3(0x03f00000, "RDRAM_CONFIG_REG or RDRAM_DEVICE_TYPE_REG",)               \
        CASE3(0x03f00004, "RDRAM_DEVICE_ID_REG",)               \
        CASE3(0x03f00008, "RDRAM_DELAY_REG",)               \
        CASE3(0x03f0000C, "RDRAM_MODE_REG",)               \
        CASE3(0x03f00010, "RDRAM_REF_INTERVAL_REG",)               \
        CASE3(0x03f00014, "RDRAM_REF_ROW_REG",)               \
        CASE3(0x03f00018, "RDRAM_RAS_INTERVAL_REG",)               \
        CASE3(0x03f0001c, "RDRAM_MIN_INTERVAL_REG",)               \
        CASE3(0x03f00020, "RDRAM_ADDR_SELECT_REG",)               \
        CASE3(0x03f00024, "RDRAM_DEVICE_MANUF_REG",)               \
        CASE4(0x03f00028, 0x03ffffff, "RDRAM_REF_ROW_REG",)               \
        CASE4(0x04000000, 0x04000FFF, "SP_DMEM read/write",)               \
        CASE4(0x04001000, 0x04001FFF, "SP_IMEM read/write",)               \
        CASE4(0x04002000, 0x0403FFFF, "unused",)               \
        CASE3(0x04040000, "SP_MEM_ADDR_REG",)               \
        CASE3(0x04040004, "SP_DRAM_ADDR_REG",)               \
        CASE3(0x04040008, "SP_RD_LEN_REG",)               \
        CASE3(0x0404000c, "SP_WR_LEN_REG",)               \
        CASE3(0x04040010, "SP_STATUS_REG",)               \
        CASE3(0x04040014, "SP_DMA_FULL_REG",)               \
        CASE3(0x04040018, "SP_DMA_BUSY_REG",)               \
        CASE3(0x0404001c, "SP_SEMAPHORE_REG",)               \
        CASE4(0x04040020, 0x0407FFFF, "unused",)               \
        CASE3(0x04080000, "SP_PC_REG",)               \
        CASE3(0x04080004, "SP_IBIST_REG",)               \
        CASE4(0x04080008, 0x040FFFFF, "unused",)      \
        CASE3(0x04100000, "DPC_START_REG",)               \
        CASE3(0x04100004, "DPC_END_REG",)               \
        CASE3(0x04100008, "DPC_CURRENT_REG",)               \
        CASE3(0x0410000c, "DPC_STATUS_REG",)               \
        CASE3(0x04100010, "DPC_CLOCK_REG",)               \
        CASE3(0x04100014, "DPC_BUFBUSY_REG",)               \
        CASE3(0x04100018, "DPC_PIPEBUSY_REG",)               \
        CASE3(0x0410001c, "DPC_TMEM_REG",)               \
        CASE4(0x04100020, 0x041FFFFF, "unused",)               \
        CASE3(0x04200000, "DPS_TBIST_REG",)               \
        CASE3(0x04200004, "DPS_TEST_MODE_REG",)               \
        CASE3(0x04200008, "DPS_BUFTEST_ADDR_REG",)               \
        CASE3(0x0420000c, "DPS_BUFTEST_DATA_REG",)               \
        CASE4(0x04200010, 0x042FFFFF, "unused",)               \
        CASE3(0x04400000, "VI_STATUS_REG or VI_CONTROL_REG",)               \
        CASE3(0x04400004, "VI_ORIGIN_REG or VI_DRAM_ADDR_REG",)               \
        CASE3(0x04400008, "VI_WIDTH_REG or VI_H_WIDTH_REG",)               \
        CASE3(0x0440000c, "VI_INTR_REG or VI_V_INTR_REG",)               \
        CASE3(0x04400010, "VI_CURRENT_REG or VI_V_CURRENT_LINE_REG",)               \
        CASE3(0x04400014, "VI_BURST_REG or VI_TIMING_REG",)               \
        CASE3(0x04400018, "VI_V_SYNC_REG",)               \
        CASE3(0x0440001c, "VI_H_SYNC_REG",)               \
        CASE3(0x04400020, "VI_LEAP_REG or VI_H_SYNC_LEAP_REG",)               \
        CASE3(0x04400024, "VI_H_START_REG or VI_H_VIDEO_REG",)               \
        CASE3(0x04400028, "VI_V_START_REG or VI_V_VIDEO_REG",)               \
        CASE3(0x0440002c, "VI_V_BURST_REG",)               \
        CASE3(0x04400030, "VI_X_SCALE_REG",)               \
        CASE3(0x04400034, "VI_Y_SCALE_REG",)               \
        CASE4(0x04400038, 0x044FFFFF, "VI_Y_SCALE_REG",)               \
        CASE3(0x04500000, "AI_DRAM_ADDR_REG",)               \
        CASE3(0x04500004, "AI_LEN_REG",)               \
        CASE3(0x04500008, "AI_CONTROL_REG",)               \
        CASE3(0x0450000c, "AI_STATUS_REG",)               \
        CASE3(0x04500010, "AI_DACRATE_REG",)               \
        CASE3(0x04500014, "AI_BITRATE_REG",)               \
        CASE4(0x04500018, 0x045FFFFF, "unused",)               \
        CASE3(0x04600000, "PI_DRAM_ADDR_REG",)               \
        CASE3(0x04600004, "PI_CART_ADDR_REG",)               \
        CASE3(0x04600008, "PI_RD_LEN_REG",)               \
        CASE3(0x0460000c, "PI_WR_LEN_REG",)               \
        CASE3(0x04600010, "PI_STATUS_REG",)               \
        CASE3(0x04600014, "PI_BSD_DOM1_LAT_REG or PI_DOMAIN1_REG",)               \
        CASE3(0x04600018, "PI_BSD_DOM1_PWD_REG",)               \
        CASE3(0x0460001c, "PI_BSD_DOM1_PGS_REG",)               \
        CASE3(0x04600020, "PI_BSD_DOM1_RLS_REG",)               \
        CASE3(0x04600024, "PI_BSD_DOM2_LAT_REG or PI_DOMAIN2_REG",)               \
        CASE3(0x04600028, "PI_BSD_DOM2_PWD_REG",)               \
        CASE3(0x0460002c, "PI_BSD_DOM2_PGS_REG",)               \
        CASE3(0x04600030, "PI_BSD_DOM2_RLS_REG",)               \
        CASE4(0x04600034, 0x046FFFFF, "unused",)               \
        CASE3(0x04700000, "RI_MODE_REG",)               \
        CASE3(0x04700004, "RI_CONFIG_REG",)               \
        CASE3(0x04700008, "RI_CURRENT_LOAD_REG",)               \
        CASE3(0x0470000c, "RI_SELECT_REG",)               \
        CASE3(0x04700010, "RI_REFRESH_REG or RI_COUNT_REG",)               \
        CASE3(0x04700014, "RI_LATENCY_REG",)               \
        CASE3(0x04700018, "RI_RERROR_REG",)               \
        CASE3(0x0470001c, "RI_WERROR_REG",)               \
        CASE3(0x04700020, 0x047FFFFF, "unused",)               \
        CASE3(0x04800000, "SI_DRAM_ADDR_REG",)\
        CASE3(0x04800004, "SI_PIF_ADDR_RD64B_REG",)\
        CASE3(0x04800008, "reserved",)\
        CASE3(0x0480000c, "reserved",)\
        CASE3(0x04800010, "SI_PIF_ADDR_WR64B_REG",)\
        CASE3(0x04800014, "reserved",)\
        CASE3(0x04800018, "SI_STATUS_REG",)\
        CASE4(0x0480001C, 0x048FFFFF, "unused")\
        CASE4(0x04900000, 0x04FFFFFF, "unused")\
        CASE4(0x05000000, 0x05FFFFFF, "64dd regs")\
        CASE4(0x06000000, 0x07FFFFFF, "64dd rom")\
        CASE4(0x08000000, 0x0FFFFFFF, "cart sram")\
        CASE4(0x10000000, 0x17ffffff, "cart rom")      \
        CASE3(0x18000000,"gio interrupt")               \
        CASE3(0x18000400,"gio sync")               \
        CASE3(0x18000800,"gio cartsync")               \
        CASE4(0x1FC00000, 0x1FC007BF, "pifrom", ret = n64->pifrom[addr & 0x7ff];)               \
        CASE4(0x1FC007C0, 0x1FC007FF, "pifram", ret = n64->pifram[addr & 0x7ff - 0x7c0];)               \
        CASE4(0x1FC00800, 0x1FCFFFFF, "unused",)               \
        CASE4(0x1FD00000, 0x7FFFFFFF, "cart domain 1 addr 3",)               \
        CASE4(0x80000000, 0x9FFFFFFF, "KSEG0",)               \
        CASE4(0xa0000000, 0xbFFFFFFF, "KSEG1",)               \
        CASE4(0xc0000000, 0xdFFFFFFF, "KSSEG",)               \
        CASE4(0xe0000000, 0xfFFFFFFF, "KSEG3",)               \
    }\
    return ret;\
}
#undef CASE3
#undef CASE4

#define read8(addr)
#define read16(addr)
#define read32(addr)
#define read64(addr)
#define read128(addr)
#define write8(addr, data)
#define write16(addr, data)
#define write32(addr, data)
#define write64(addr, data)
#define write128(addr, data)




#define pc n64->pc.ud[0]

#define rd ((opcode >> 11) & 0x1f)
#define rs ((opcode >> 21) & 0x1f)
#define rt ((opcode >> 16) & 0x1f)
#define sa ((opcode >> 6) & 0x1f)
#define funct (opcode & 0x3f)

#define rd32u n64->gpr[rd].uw[0]
#define rd32s n64->gpr[rd].sw[0]
#define rs32u n64->gpr[rs].uw[0]
#define rs32s n64->gpr[rs].sw[0]
#define rt32u n64->gpr[rt].uw[0]
#define rt32s n64->gpr[rt].sw[0]

#define rd64u n64->gpr[rd].uw[0]
#define rd64s n64->gpr[rd].sw[0]
#define rs64u n64->gpr[rs].uw[0]
#define rs64s n64->gpr[rs].sw[0]
#define rt64u n64->gpr[rt].uw[0]
#define rt64s n64->gpr[rt].sw[0]

#define hi32u n64->hi.uw[0]
#define hi32s n64->hi.sw[0]
#define hi64u n64->hi.ud[0]
#define hi64s n64->hi.sd[0]
#define lo32u n64->lo.uw[0]
#define lo32s n64->lo.sw[0]
#define lo64u n64->lo.ud[0]
#define lo64s n64->lo.sd[0]

#define imm16 ((s16)opcode)
#define uimm16 ((u16)opcode)
#define branchoffset (((s32)imm16) << 2)
#define target ((pc & 0xf0000000) | ((opcode & 0x03ffffff) << 2))


#define branch(cond,likely,link) if(cond){n64->indelay = true; n64->nextpc = pc + 4 + branchoffset;}else{if(likely)pc += 4;} if(link) n64->gpr[31].ud[0] = pc + 8;
#define jump() n64->indelay = true; n64->nextpc = target;
#define exception()
#define invalid()

#define CASE2(op, debugstr, ...) case __COUNTER__ - baseint: dbg(debugstr); op; break;

void _interpspecial(struct n64* n64, u32 opcode){
    static const int baseint = __COUNTER__ + 1;
    switch (_special) {
        CASE2(rd64s = rt32s << sa, "sll",)
        CASE2(invalid(), "invalid",)
        CASE2(rd64u = rt32u >> sa, "srl",)
        CASE2(rs64s = rt32s >> sa, "sra",)
        CASE2(rd64s = rt32s << (rs32s & 0x1f), "sllv",)
        CASE2(invalid(), "invalid",)
        CASE2(rd64s = rt32u >> (rs32u & 0x1f), "SRLV",)
        CASE2(rd64s = rt32s >> (rs32s & 0x1f), "SRAV",)
        CASE2(, "jr",)
        CASE2(, "jalr",)
        CASE2(invalid(), "invalid",)
        CASE2(invalid(), "invalid",)
        CASE2(, "syscall",)
        CASE2(, "break",)
        CASE2(, "invalid",)
        CASE2(, "sync",)
        CASE2(, "mfhi",)
        CASE2(, "mthi",)
        CASE2(, "mflo",)
        CASE2(, "mtlo",)
        CASE2(rd64u = rt64u << (rs64u & 0x3f), "dsllv",)
        CASE2(invalid(), "invalid",)
        CASE2(rd64u = rt64u >> (rs64u & 0x2f), "dsrlv",)
        CASE2(rd64s = rt64s << (rs64u & 0x3f), "dsrav",)
        CASE2(, "mult",)
        CASE2(, "multu",)
        CASE2(lo64s = rs32s / rt32s;hi64s = rs32s % rt32s;, "div",)
        CASE2(lo64u = rs32u / rt32u; hi64u = rs32u % rt32u;, "divu",)
        CASE2(lo64u = (s128)rs64s * (s128)rt64s; hi64u = ((s128)rs64s * (s128)rt64s) >> 64, "dmult",)
        CASE2(lo64u = (s128)rs64u * (s128)rt64u; hi64u = ((s128)rs64u * (s128)rt64u) >> 64, "dmultu",)
        CASE2(lo64s = rs64s / rt64s; hi64s = rs64s % rt64s;, "ddiv",)
        CASE2(lo64u = rs64u / rt64u; hi64u = rs64u % rt64u;, "ddivu",)
        CASE2(rd64s = rs32s + rt32s;,"add",)
        CASE2(rd64s = rs32s + rt32s;,"addu",)
        CASE2(rd64s = rs32s - rt32s;,"sub",)
        CASE2(rd64s = rs32s - rt32s;,"subu",)
        CASE2(rd64u = rs64u & rt64u;,"and",)
        CASE2(rd64u = rs64u | rt64u;,"or",)
        CASE2(rd64u = (rs64u | rt64u) & ~(rs64u & rt64u),"xor",)
        CASE2(rd64u = ~(rt64u | rs64u),"nor",)
        CASE2(invalid(),"invalid",)
        CASE2(invalid(),"invalid",)
        CASE2(rd64s = (rs64s < rt64s) ? 1 : 0,"slt",)
        CASE2(rd64s = (rs64u < rt64u) ? 1 : 0,"sltu",)
        CASE2(rd64s = rs64s + rt64s;,"dadd",)
        CASE2(rd64s = rs64s + rt64s;,"daddu",)
        CASE2(rd64s = rs64s = rt64s;,"dsub",)
        CASE2(rd64s = rs64s = rt64s;,"dsubu",)
        CASE2(,"tge",)
        CASE2(,"tgeu",)
        CASE2(,"tlt",)
        CASE2(,"tltu",)
        CASE2(,"teq",)
        CASE2(invalid(),"invalid",)
        CASE2(,"tne",)
        CASE2(invalid(),"invalid",)
        CASE2(rd64u = rt64u << sa;,"dsll",)
        CASE2(invalid(),"invalid",)
        CASE2(rd64u = rt64u >> sa;,"dsrl",)
        CASE2(rd64s = rt64s >> sa;,"dsra",)
        CASE2(rd64u = rt64u << (32 + sa);,"dsll32",)
        CASE2(invalid(),"invalid",)
        CASE2(rd64u = rt64u >> (32 + sa);,"dsrl32",)
        CASE2(rs64s = rt64s >> (sa + 32);,"dsra32",)
    }
}

void _interpregimm(struct n64* n64, u32 opcode){
    static const int baseint = __COUNTER__ + 1;
    switch (_regimm) {
        CASE2(branch(rs64s < 0, false, false), "bltz",)
        CASE2(branch(rs64s >= 0, false, false);,"bgez")
        CASE2(branch(rs64s < 0, true, false);,"bltzl")
        CASE2(branch(rs64s >= 0, true, false);,"bgezl")
        CASE2(,"invalid")
        CASE2(,"invalid")
        CASE2(,"invalid")
        CASE2(,"invalid")
        CASE2(,"tgei")
        CASE2(,"tgeiu")
        CASE2(,"tlti")
        CASE2(,"tltiu")
        CASE2(,"teqi")
        CASE2(,"invalid")
        CASE2(,"tnei")
        CASE2(,"invalid")
        CASE2(branch(rs64s < 0, false, true);,"bltzal")
        CASE2(branch(rs64s >= 0, false, true);,"bgezal")
        CASE2(branch(rs64s < 0, true, true);,"bltzall")
        CASE2(branch(rs64s >= 0, true, true);,"bgezall")
        case 20 ... 31: invalid(); dbg("invalid"); break;
    }
}

void _interpcop0(struct n64* n64, u32 opcode){
    switch(rs){
        case 0: break; //mfc0
        case 1: break; //dmfc0
        case 2: break; //cfc0
        case 3: break; //invalid
        case 4: break; //mtc0
        case 5: break; //dmtc0
        case 6: break; //ctc0
        case 7: break; //invalid
        case 8: switch(rt) {
                case 0: break;//bcf0
                case 1: break;//bct0
                case 2: break;//bcfl0
                case 3: break;//bctl0
                case 4 ... 31: break;//invalid
            }//bc
        case 9 ... 15: break; //invalid
        case 16 ... 31: switch(funct){
                case 0: break; //invalid
                case 1: break; //tlbr
                case 2: break; //tlbwi
                case 3: break; //invalid
                case 4: break; //invalid
                case 5: break; //invalid
                case 6: break; //tlbwr
                case 7: break; //invalid
                case 8: break; //tlbp
                case 9 ... 23: break; //invalid
                case 24: break; //eret
                case 25 ... 63: break; //invalid
            }
    }
}

void _interpcop1(struct n64* n64, u32 opcode) {
    switch(rs){
        case 0: break; //mfc1
        case 1: break; //dmfc1
        case 2: break; //cfc1
        case 3: break; //invalid
        case 4: break; //mtc1
        case 5: break; //dmtc1
        case 6: break; //ctc1
        case 7: break; //invalid
        case 8: switch(rt) {
                case 0: break;//bcf1
                case 1: break;//bct1
                case 2: break;//bcfl1
                case 3: break;//bctl1
                case 4 ... 31: break;//invalid
            }//bc
        case 9 ... 15: break; //invalid
        case 16 ... 31:
            switch(funct) {
                case 0: break; //add
                case 1: break; //sub
                case 2: break; //mul
                case 3: break; //div
                case 4: break; //sqrt
                case 5: break; //abs
                case 6: break; //mov
                case 7: break; //neg
                case 8: break; //round.l
                case 9: break; //trunc.l
                case 10: break; //ceil.l
                case 11: break; //floor.l
                case 12: break; //round.w
                case 13: break;//trunc.w
                case 14: break;//ceil.w
                case 15: break;//floor.w
                case 16 ... 31: break; //invalid
                case 32: break; //cvt.s
                case 33: break;//cvt.d
                case 34: break;//invalid
                case 35: break;//invalid
                case 36: break;//cvt.w
                case 37: break;//cvt.l
                case 38 ... 47: //invalid
                case 48: break; //c.f
                case 49: break; //c.un
                case 50: break; //c.eq
                case 51: break; //c.ueq
                case 52: break; //c.olt
                case 53: break; //c.ult
                case 54: break; //c.ole
                case 55: break; //c.ule
                case 56: break; //c.sf
                case 57: break; //c.ngle
                case 58: break; //c.seq
                case 59: break; //c.ngl
                case 60: break; //c.lt
                case 61: break; //c.nge
                case 62: break; //c.le
                case 63: break; //c.ngt
            } break;
    }
}
void _interpcop2(struct n64* n64, u32 opcode){}

void interpret(struct n64 *n64, u32 opcode) {

    static const int baseint = __COUNTER__ + 1;
    switch (_op) {
        CASE2(_interpspecial(n64,opcode);, "special")
        CASE2(_interpregimm(n64,opcode);, "regimm")
        CASE2(jump(),"j")
        CASE2(,"jal")
        CASE2(branch(rd64u == rt64u,false,false),"beq")
        CASE2(branch(rd64u != rt64u,false,false),"bne")
        CASE2(branch(rs64s <= 0, false, false),"BLEZ")
        CASE2(branch(rs64s > 0, false, false),"BGTZ")
        CASE2(rt64s = rs32s + imm16;,"addi")
        CASE2(rt64s = rs32s + imm16;,"addiu")
        CASE2(rt64s = (rs64s < imm16) ? 1 : 0;,"slti")
        CASE2(rt64s = (rs64u < uimm16) ? 1 : 0;,"sltiu")
        CASE2(rt64u = rs64u & uimm16;;,"andi")
        CASE2(,"ori")
        CASE2(,"xori")
        CASE2(,"lui")
        CASE2(_interpcop0(n64, opcode);, "cop0")
        CASE2(_interpcop1(n64, opcode);, "cop1")
        CASE2(_interpcop2(n64, opcode);, "cop2")
        CASE2(invalid(),"invalid")
        CASE2(branch(rs64u == rt64u, true, false),"beql")
        CASE2(branch(rs64s >= rt64u, true, false),"bnel")
        CASE2(branch(rs64s <= 0, true, false),"blezl")
        CASE2(branch(rs64s > 0, true, false),"bgtzl")
        CASE2(rt64s = rs64s + imm16;,"daddi")
        CASE2(rt64s = rs64s + imm16;,"daddiu")
        CASE2(,"ldl")
        CASE2(,"ldr")
        CASE2(,"invalid")
        CASE2(,"invalid")
        CASE2(,"invalid")
        CASE2(,"invalid")
        CASE2(,"lb")
        CASE2(,"lh")
        CASE2(,"lwl")
        CASE2(,"lw")
        CASE2(,"lbu")
        CASE2(,"lhu")
        CASE2(,"lwr")
        CASE2(,"lwu")
        CASE2(,"sb")
        CASE2(,"sh")
        CASE2(,"swl")
        CASE2(,"sw")
        CASE2(,"sdl")
        CASE2(,"sdr")
        CASE2(,"swr")
        CASE2(,"cache")
        CASE2(,"ll")
        CASE2(,"lwc1")
        CASE2(,"lwc2")
        CASE2(,"invalid")
        CASE2(,"lld")
        CASE2(,"ldc1")
        CASE2(,"ldc2")
        CASE2(,"ld")
        CASE2(,"sc")
        CASE2(,"swc1")
        CASE2(,"swc2")
        CASE2(,"invalid")
        CASE2(,"scd")
        CASE2(,"sdc1")
        CASE2(,"sdc2")
        CASE2(,"sd")
    }
}