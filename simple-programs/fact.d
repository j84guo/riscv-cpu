
fact.elf:     file format elf32-littleriscv


Disassembly of section .text:

01000000 <main>:
 1000000:	fe010113          	addi	sp,sp,-32
 1000004:	00112e23          	sw	ra,28(sp)
 1000008:	00012623          	sw	zero,12(sp)
 100000c:	01c0006f          	j	1000028 <main+0x28>
 1000010:	00c12503          	lw	a0,12(sp)
 1000014:	03c000ef          	jal	ra,1000050 <factorial>
 1000018:	00a12423          	sw	a0,8(sp)
 100001c:	00c12783          	lw	a5,12(sp)
 1000020:	00178793          	addi	a5,a5,1
 1000024:	00f12623          	sw	a5,12(sp)
 1000028:	00c12703          	lw	a4,12(sp)
 100002c:	00900793          	li	a5,9
 1000030:	fee7d0e3          	bge	a5,a4,1000010 <main+0x10>
 1000034:	00812503          	lw	a0,8(sp)
 1000038:	090000ef          	jal	ra,10000c8 <test>
 100003c:	00050793          	mv	a5,a0
 1000040:	00078513          	mv	a0,a5
 1000044:	01c12083          	lw	ra,28(sp)
 1000048:	02010113          	addi	sp,sp,32
 100004c:	00008067          	ret

01000050 <factorial>:
 1000050:	fd010113          	addi	sp,sp,-48
 1000054:	02112623          	sw	ra,44(sp)
 1000058:	00a12623          	sw	a0,12(sp)
 100005c:	00012e23          	sw	zero,28(sp)
 1000060:	00012c23          	sw	zero,24(sp)
 1000064:	00c12783          	lw	a5,12(sp)
 1000068:	00079663          	bnez	a5,1000074 <factorial+0x24>
 100006c:	00100793          	li	a5,1
 1000070:	0480006f          	j	10000b8 <factorial+0x68>
 1000074:	00012e23          	sw	zero,28(sp)
 1000078:	0300006f          	j	10000a8 <factorial+0x58>
 100007c:	00c12783          	lw	a5,12(sp)
 1000080:	fff78793          	addi	a5,a5,-1
 1000084:	00078513          	mv	a0,a5
 1000088:	fc9ff0ef          	jal	ra,1000050 <factorial>
 100008c:	00050713          	mv	a4,a0
 1000090:	01812783          	lw	a5,24(sp)
 1000094:	00e787b3          	add	a5,a5,a4
 1000098:	00f12c23          	sw	a5,24(sp)
 100009c:	01c12783          	lw	a5,28(sp)
 10000a0:	00178793          	addi	a5,a5,1
 10000a4:	00f12e23          	sw	a5,28(sp)
 10000a8:	01c12703          	lw	a4,28(sp)
 10000ac:	00c12783          	lw	a5,12(sp)
 10000b0:	fcf746e3          	blt	a4,a5,100007c <factorial+0x2c>
 10000b4:	01812783          	lw	a5,24(sp)
 10000b8:	00078513          	mv	a0,a5
 10000bc:	02c12083          	lw	ra,44(sp)
 10000c0:	03010113          	addi	sp,sp,48
 10000c4:	00008067          	ret

010000c8 <test>:
 10000c8:	fe010113          	addi	sp,sp,-32
 10000cc:	00112e23          	sw	ra,28(sp)
 10000d0:	00a12623          	sw	a0,12(sp)
 10000d4:	00c12703          	lw	a4,12(sp)
 10000d8:	000597b7          	lui	a5,0x59
 10000dc:	98078793          	addi	a5,a5,-1664 # 58980 <main-0xfa7680>
 10000e0:	00f71863          	bne	a4,a5,10000f0 <test+0x28>
 10000e4:	024000ef          	jal	ra,1000108 <pass>
 10000e8:	00050793          	mv	a5,a0
 10000ec:	00c0006f          	j	10000f8 <test+0x30>
 10000f0:	024000ef          	jal	ra,1000114 <fail>
 10000f4:	00050793          	mv	a5,a0
 10000f8:	00078513          	mv	a0,a5
 10000fc:	01c12083          	lw	ra,28(sp)
 1000100:	02010113          	addi	sp,sp,32
 1000104:	00008067          	ret

01000108 <pass>:
 1000108:	00100793          	li	a5,1
 100010c:	00078513          	mv	a0,a5
 1000110:	00008067          	ret

01000114 <fail>:
 1000114:	00000793          	li	a5,0
 1000118:	00078513          	mv	a0,a5
 100011c:	00008067          	ret

Disassembly of section .comment:

00000000 <.comment>:
   0:	3a434347          	fmsub.d	ft6,ft6,ft4,ft7,rmm
   4:	2820                	fld	fs0,80(s0)
   6:	29554e47          	fmsub.s	ft8,fa0,fs5,ft5,rmm
   a:	3820                	fld	fs0,112(s0)
   c:	332e                	fld	ft6,232(sp)
   e:	302e                	fld	ft0,232(sp)
	...

Disassembly of section .riscv.attributes:

00000000 <.riscv.attributes>:
   0:	1941                	addi	s2,s2,-16
   2:	0000                	unimp
   4:	7200                	flw	fs0,32(a2)
   6:	7369                	lui	t1,0xffffa
   8:	01007663          	bgeu	zero,a6,14 <main-0xffffec>
   c:	0000000f          	fence	unknown,unknown
  10:	7205                	lui	tp,0xfffe1
  12:	3376                	fld	ft6,376(sp)
  14:	6932                	flw	fs2,12(sp)
  16:	7032                	flw	ft0,44(sp)
  18:	0030                	addi	a2,sp,8
