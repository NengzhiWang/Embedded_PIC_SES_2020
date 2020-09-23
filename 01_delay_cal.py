# delay time calculater for project: 01_blink_delay_asm


def Inst_Num(n0, n1, n2, offset):
    return 3 * n0 * n1 * n2 + 4 * n0 * n1 + 4 * n0 + 5 + offset


def Calculate_N(Instruction_Num, offset):
    inst_diff = abs(Inst_Num(0, 0, 0, offset) - Instruction_Num)
    N = 0
    for i in range(0xffffff):
        n0 = i % 256
        n1 = (i // 256) % 256
        n2 = (i // 65536) % 256
        inst_period = Inst_Num(n0, n1, n2, offset)
        if abs(inst_period - Instruction_Num) <= inst_diff:
            N = i
            inst_diff = abs(inst_period - Instruction_Num)
    n0 = N % 256
    n1 = (N // 256) % 256
    n2 = (N // 65536) % 256
    return n0, n1, n2


inst_freq = 1e6
# input the frequency of your PIC, default 1Mhz
loop_time = .5
# input the time you want to delay
loop_offset = 4
# input the instruction number out of delay
inst_num = int(loop_time * inst_freq)
n0, n1, n2 = Calculate_N(inst_num, loop_offset)

print('Required Instruction Num', inst_num)
print('n0=', hex(n0))
print('n1=', hex(n1))
print('n2=', hex(n2))

print('Accuracy Instruction', Inst_Num(n0, n1, n2, loop_offset))
