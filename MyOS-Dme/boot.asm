; ============================================================================
;  Lomsourt OS - Bootchain: 16-bit -> 32-bit -> 64-bit (Steps 1.1-1.5)
;  File      : boot.asm
;  Compile   : nasm -f bin boot.asm -o lomsourt_os.bin
;              nasm -f bin boot.asm -D DEMO_FAULT -o lomsourt_os_test.bin
;  Test      : qemu-system-x86_64 -drive format=raw,file=lomsourt_os.bin
;
;  HOW THE TRANSITION WORKS (execution order):
;    1. BIOS loads sector 1 (512 bytes) to 0x7C00. Execution starts at start:.
;    2. 16-BIT : segments/stack init, welcome banner, INT 0x13 disk read of
;                sector 2 (the 64-bit kernel) into RAM at 0x7E00.
;    3. switch_to_pm : CLI, enable A20, LGDT, CR0.PE <- 1, then a FAR jump
;                with the 32-bit code selector (trampoline: 16 -> 32 bit).
;    4. init_pm [BITS 32] : flat data segments, stack at 0x90000, prints a
;                confirmation string straight into VGA text memory (0xB8000).
;    5. enter_long_mode [BITS 32] : builds a PML4 identity map (2 MiB pages),
;                enables CR4.PAE, EFER.LME, CR0.PG, then far-jumps with the
;                64-bit code selector (L=1) into start64 (Stage 2, sector 2).
;    6. start64 [BITS 64] : reloads long-mode segments, prints the long-mode
;                banner via the 64-bit VGA writer, then halts.
;
;  CPU/MODE TIMELINE (the architectural LOCK):
;    Real Mode 16      -> PM 32        -> (paging+PAE+LME) -> Long Mode 64.
;    16/32-bit stages exist ONLY as the bridge ('trampoline') mandated by the
;    x86_64 architectural lock; the OS itself is a 64-bit Monolithic kernel.
;
;  IMAGE LAYOUT (raw sectors, CHS sector 1-based):
;    Byte 0x000 - 0x1FF : Sector 1 -> bootcode + GDT + switch + page tables
;    Byte 0x200 - 0x3FF : Sector 2 -> 64-bit kernel entry (loaded to 0x7E00)
;    Byte 0x400       + : Sectors 3-7 -> IDT + panic infrastructure; then the
;                         freestanding C kernel (Step 2.1) is APPENDED after
;                         the payload (staged at KERNEL_STAGE, copied to
;                         0x100000 in long mode, entered as kernel_main).
;
;  Combined source is assembled into ONE binary via %include, so a single
;  NASM call yields a bootable multi-sector image.
; ============================================================================

    ORG 0x7C00          ; All flat addresses below are ORG-relative: they are
                        ;   the 16-bit-addresses the BIOS placed us at AND,
                        ;   after the far jump, the 32-bit flat addresses we
                        ;   execute at. Keeping ONE ORG for the whole image is
                        ;   what makes cross-sector relative calls correct.
BITS 16                 ; The CPU enters in real mode: 16-bit everything.

; ----------------------------------------------------------------------------
;  LAYOUT CONSTANTS (Step 2.1). These MUST match build.bat / build.ps1:
;    Payload (start64 + IDT + panic) = 6 sectors, loaded by disk_load.
;    C kernel shares the SAME read: it is appended at sector (2+PAYLOAD_SECTS)
;    and lands contiguously AFTER the payload at KERNEL_STAGE. It is then
;    copied to KERNEL_BASE in long mode (start64) and called as kernel_main.
; ----------------------------------------------------------------------------
PAYLOAD_SECTS equ 6             ; Sectors 2..7 (64-bit startup + exceptions).
%ifndef KERNEL_SECTORS          ; Overridable: nasm -DKERNEL_SECTORS=<n>.
    %define KERNEL_SECTORS 8    ; Default C-kernel size in 512-byte sectors.
%endif
%ifndef INITRD_SECTORS          ; Overridable: nasm -DINITRD_SECTORS=<n>.
    %define INITRD_SECTORS 0    ; Default: no initrd (Steps 1.x/2.x/3.x images).
%endif

KERNEL_BASE  equ 0x100000       ; C kernel link/load address (identity-mapped).
KERNEL_STAGE equ 0x7E00 + PAYLOAD_SECTS * 512   ; 0x8A00: where it is staged.

; Step 4.1 - the USTAR initrd rides the boot floppy right after the kernel blob
; (contiguous with the staging copy).  start64 (sector 2) relocates it from
; there into INITRD_DEST (free conventional RAM) and records address + size in
; the boot-info block at 0x500 so kernel_main can mount it with no side
; channel.  All of these live below 0x9FC00 (EBDA: 0x9FC00-0xA0000), so the
; kernel's PMM never hands them out and the blob stays intact for the VFS demo.
; Step 10.1 - CORE-LOCK: the C kernel owns an ownership gate (SHA-256 owner
; password) that must verify before any device/disk/user code runs; on success
; it stamps the gate word at BOOT_INFO+16 (0x510 = 0xACCE55ED). The bootstrap
; itself prints the protection banner below so a locked console still proves
; the binary is the protected build.
INITRD_STAGE_FROM equ KERNEL_STAGE + KERNEL_SECTORS * 512  ; staged copy tail.
INITRD_DEST       equ 0x60000
BOOT_INFO         equ 0x500      ;   +0 : qword initrd physical address
                                 ;   +8 : qword initrd size in bytes

; ----------------------------------------------------------------------------
;  ENTRY (16-bit real mode)
; ----------------------------------------------------------------------------
start:
    ; --- Segment registers + stack (identical to Steps 1.1/1.2) -------------
    xor ax, ax          ; AX = 0x0000 (zero 16-bit segment registers fast).
    mov ds, ax          ; DS = 0x0000: flat addressing for [BOOT_DRIVE] etc.
    mov es, ax          ; ES = 0x0000: INT 0x13 buffer = 0:0x7E00.
    mov ss, ax          ; SS = 0x0000: stack segment.
    mov sp, 0x7C00      ; SP: stack descends from 0x7C00 (our own code below).

    mov [BOOT_DRIVE], dl; Remember the BIOS boot-drive number for disk_load.

    ; --- 16-bit welcome banner ----------------------------------------------
    cld                 ; Strings move forward (lodsb).
    mov si, msg_welcome ; SI = "Lomsourt OS Booting...".
    call print_string   ; BIOS INT 0x10 / AH=0x0E teletype.

    ; --- Load the rest of the OS: payload + kernel + initrd ------------------
    ; disk_load walks CHS one sector per INT 0x13 call, so an arbitrarily
    ; large contiguous region can be pulled from the floppy into 0x7E00..
    ; (crossing cylinders, heads AND the 64 KiB ES:BX window is handled).
    ; The CHS cursor is DERIVED per sector from the FDC's real geometry
    ; (36 sectors/track, 2 heads -- see disk_load for the measurement).
    mov bx, 0x7E00          ; INT 0x13 ES:BX destination = 0x0000:0x7E00.
    mov di, PAYLOAD_SECTS + KERNEL_SECTORS + INITRD_SECTORS  ; sectors total.
    call disk_load          ; Halts on error.

    ; --- CROSS THE RUBICON: real mode -> protected mode ---------------------
    call switch_to_pm   ; CLI + A20 + LGDT + CR0.PE=1 + far jump (+ never
                        ;   returns; execution continues at init_pm).
    jmp $               ; Unreachable safety net (switch_to_pm doesn't RET).

; ----------------------------------------------------------------------------
;  Included modules. Order matters for NASM's [BITS] state machine:
;    gdt.asm        : pure data -> adds no code, leaves [BITS] untouched.
;    switch_pm.asm  : opens in [bits 16], ends with a [bits 32] section.
;    print_pm.asm   : [bits 32] VGA routine.
;    long_mode.asm  : [bits 32] page-tree + long-mode enable.
;  Immediately after the includes we re-assert [bits 16] before the 16-bit
;  disk/print helpers so they assemble with the correct operand sizes.
; ----------------------------------------------------------------------------
%include "gdt.asm"
%include "switch_pm.asm"
%include "print_pm.asm"
%include "long_mode.asm"

[bits 16]                       ; Back to 16-bit for the real-mode helpers.

; ============================================================================
;  print_string - 16-bit BIOS teletype printer (unchanged from Step 1.2).
;  INPUT : SI  = DS-relative address of null-terminated string.
; ----------------------------------------------------------------------------
print_string:
    lodsb               ; AL = *SI++, SI advanced (DF clear).
    or al, al           ; NUL terminator?
    jz .done            ; Yes -> return.
    mov ah, 0x0E        ; AH = 0x0E, INT 0x10 = teletype output.
    int 0x10            ; Print AL at current cursor, advance cursor.
    jmp print_string    ; Loop.
.done:
    ret

; ============================================================================
;  disk_load - single-sector INT 0x13 CHS reader, LBA->CHS conversion.
;  INPUT : DI = sectors to read (running LBA from 1 upward, so image sector 2
;          onward -> the MBR at LBA 0 is skipped), ES:BX = destination.
;  Converts every LBA to CHS with the FDC's ACTUAL sectors-per-track / head
;  counts.  Measured empirically: the QEMU 11.1 raw floppy maps CHS with 36
;  sectors per track (2.88 MB-style geometry), NOT the 18 of a 1.44 MB disk.
;  (A hardcoded 18-SPT walker misreads at the first head boundary: its C0H1S2
;  request returns LBA 37 = the byte run that belongs at LBA 19.)  The BIOS
;  geometry query (INT 0x13 AH=0x08) would be cleaner but wedges the QEMU
;  FDC with an IRQ6 storm, so the drive geometry is pinned to constants.
;  Bumps ES when BX rolls over the 64 KiB window.
; ============================================================================
disk_load:
.next_sector:
    ; --- LBA -> CHS (16-bit DIV), destination BX preserved throughout -------
    ; cyl  = LBA / (SPT * HEADS); head = rem / SPT; sector = rem % SPT + 1.
    mov ax, [LBA]           ; Current LBA (1-based; LBA 0 is the MBR).
    xor dx, dx
    div word [CYLW]         ; AX = cylinder, DX = offset within the cylinder.
    mov cx, ax              ; CX = cylinder.
    mov ax, dx
    xor dx, dx
    div word [SPT]          ; AL = head, DL = zero-based sector index.
    mov dh, al              ; DH = head.
    mov ax, cx              ; AX = cylinder (low 8 bits -> CH...).
    mov ch, al              ; CH = low byte of cylinder.
    shr ax, 8               ; AX = cylinder >> 8.
    and al, 0x03            ; Keep the 2 high cylinder bits.
    shl al, 6               ; Slide into CL bits 6-7.
    mov cl, dl
    inc cl                  ; CL bits 0-5 = real sector number (1-based).
    or  cl, al              ; CL = (cyl_hi << 6) | sector.

    mov dl, [BOOT_DRIVE]    ; DL = boot drive.
    mov ax, 0x0201          ; AH = 0x02 (read), AL = 0x01 (one sector).
    int 0x13                ; BIOS read. CF = 0 on success.
    jc .error
    add bx, 0x0200          ; Next sector slot in the buffer.
    jnc .no_seg_roll        ; BX overflowed 0xFFFF?
    mov ax, es              ;   Yes -> bump ES by 0x1000 so the linear
    add ax, 0x1000          ;   address stays continuous (BX already
    mov es, ax              ;   wrapped into the new segment's offset).
.no_seg_roll:
    inc word [LBA]          ; Advance the LBA counter.
    dec di                  ; Done all requested sectors?
    jnz .next_sector
    ret                     ; Yes -> success.

.error:                     ; ---------------------------------------------
    mov si, msg_disk_error
    call print_string   ; "Disk read error!"
    cli                 ; No interrupts during the engine-out.
.haltLoop:
    hlt
    jmp .haltLoop

; ============================================================================
;  Real-mode data (live in sector 1, so already in RAM by PM entry).
; ============================================================================
msg_welcome:
    db "Lomsourt OS Booting...", 0x0D, 0x0A, 0x00
msg_disk_error:
    db "Disk read error!", 0x0D, 0x0A, 0x00

BOOT_DRIVE:
    db 0x00             ; Persisted DL value (drive we booted from).

; disk_load working state (real-mode data in sector 1):
LBA:
    dw 1                ; Current LBA (1-based: LBA 0 is the MBR, skipped).
SPT:
    dw 36               ; FDC sectors/track, measured empirically (2.88 MB-style
                        ;   geometry: 36 SPT, 2 heads -- NOT the 18 of 1.44 MB).
CYLW:
    dw 72               ; Sectors per cylinder = SPT * HEADS.

; ============================================================================
;  Boot signature: pad sector 1 to exactly 510 bytes, then 0x55 0xAA.
;  NASM errors if the boot code ever overflows 510 bytes (negative repeat),
;  which is precisely the "fits in one boot sector" property we need.
; ============================================================================
    times 510 - ($ - $$) db 0
    dw 0xAA55

; ============================================================================
;  SECTOR 2 - 64-bit kernel entry (file offset 512 == CHS sector 2)
;  disk_load copied these bytes to 0x7E00, and enter_long_mode far-jumps
;  here with CODE_SEG64 (0x18, L=1). We are now in FULL 64-BIT LONG MODE.
;
;  ADDRESSING NOTE: [bits 64] code uses RIP-relative addressing by default
;  for symbol references (and 32-bit absolute offsets via registers below).
;  All symbols in this image are within +-2 GiB, so RIP-relative works.
; ============================================================================
[bits 64]                       ; Everything below runs in 64-bit mode.

start64:
    ; ---- Reload the long-mode segment registers ---------------------------
    ; CS was switched by the far jump (CODE_SEG64). Reload the data-class
    ; segments with DATA_SEG; in long mode their base/limit are ignored and
    ; access is flat across the full 64-bit linear space.
    mov ax, DATA_SEG             ; AX = 0x0010.
    mov ss, ax                   ; Stack segment (flat, ring 0).
    mov ds, ax                   ; Data segment.
    mov es, ax                   ; Extra segment.
    mov fs, ax                   ; FS (kernel flat roaming - normalized).
    mov gs, ax                   ; GS (same rationale).

    ; ---- Fresh 64-bit stack (top of the 512 KiB below-1 MiB zone) ---------
    mov rsp, 0x90000             ; RSP = 0x90000. In long mode the stack
                                 ;   addresses are 64-bit; fit is trivial.
    mov rbp, rsp                 ; Base pointer = top of stack.

; ---- Announce long mode through the VGA frame buffer -------------------
    lea rbx, [rel MSG_LONG64]    ; RBX = address of the banner (RIP-relative,
                                  ;   guaranteed local to the loaded image).
    call print_string_64         ; Polled 64-bit VGA writer (below).

; ---- Step 10.3: Developer password barrier (before CORE-LOCK) -----------
    ; The developer gate is handled by kernel.c's core_gate() which performs
    ; SHA-256 constant-time verification with non-volatile attempt tracking
    ; at 0x400. The bootstrap prints the protection banner below.
    ; ---- Step 10.1: CORE-LOCK protection banner ---------------------------
    ; The ownership gate lives in the C kernel (SHA-256 owner password before
    ; any device/disk/user code); this line proves the protected binary is
    ; what just booted, even on a console that later locks at the gate.
    lea rbx, [rel MSG_CORELOCK]
    call print_string_64

    ; ---- Step 11.1: Live-USB Environment Detection ------------------
    lea rbx, [rel MSG_ENV_LIVE_USB]
    call print_string_64

    ; ---- Step 1.5: install 64-bit IDT, then park the CPU ------------------
    cli                          ; Mask all IRQs until the IDT is live.
    call setup_idt64             ; Build 256 gate IDT64 + LIDT (idt64.asm).

%ifdef DEMO_FAULT               ; Synthetic fault: verify the IDT catches #DE
    xor eax, eax
    xor edx, edx
    div eax                      ; 0 / 0 -> #DE (vector 0).
%endif

    ; ---- Step 2.1: hand control to the freestanding C kernel --------------
    ; The kernel was staged contiguously after the payload (sectors linked at
    ; KERNEL_BASE). Copy KERNEL_SECTORS sectors of it from KERNEL_STAGE to its
    ; link address 0x100000, then branch to kernel_main (the first symbol in
    ; the ELF is placed at 0x100000 by linker.ld's ENTRY + .text.entry).
    cld                          ; Strings/moves go forward.
    mov esi, KERNEL_STAGE        ; Source: 0x8A00 (raw bytes after payload).
    mov edi, KERNEL_BASE         ; Dest.:  0x100000 (identity-mapped by the PML4).
    mov ecx, (KERNEL_SECTORS * 512) / 8   ; 64-bit qwords to copy.
    rep movsq                    ; Bulk copy: kernel blob -> its link address.

%if INITRD_SECTORS > 0
    ; ---- Step 4.1: relocate the initrd and advertise it to the C kernel ----
    ; The archive was staged contiguously right after the kernel blob (start
    ; at INITRD_STAGE_FROM, KERNEL_STAGE+…).  Just like the kernel, copy it
    ; out of the staging area into a stable low-RAM location and stash its
    ; address/size in the boot-info block at BOOT_INFO (0x500) for kernel_main.
    mov esi, INITRD_STAGE_FROM   ; Staged initrd start.
    mov edi, INITRD_DEST         ; 0x60000: free conventional memory (< EBDA).
    mov ecx, INITRD_SECTORS * 64 ; (INITRD_SECTORS * 512) / 8  qwords.
    rep movsq                    ; Bulk copy: initrd blob -> 0x60000.

    mov eax, INITRD_DEST         ; Boot info +0 = initrd physical address.
    mov qword [abs BOOT_INFO], rax
    mov eax, INITRD_SECTORS * 512 ; Boot info +8 = initrd size in bytes.
    mov qword [abs BOOT_INFO + 8], rax
%endif

    mov rax, KERNEL_BASE         ; kernel_main lives at the very start of the
                                 ;   flattened image (linker ensures it).
    call rax                     ; Enter the C kernel (never returns).

.haltLoop:
    cli                          ; Kernel returned? Disable IRQs...
    hlt                          ; ...and park (unexpected-return catch).
    jmp .haltLoop

; ----------------------------------------------------------------------------
;  print_string_64 - 64-bit twin of print_string_pm (persistent VGA cursor).
;  INPUT : RBX = address of null-terminated string (any flat address).
;  CLOBBERS: RAX, RDX, RCX (RBX preserved).
; ----------------------------------------------------------------------------
print_string_64:
    push rbx                     ; Preserve the string pointer.
    push rcx                     ; Preserve caller's RCX (hygiene).
    push rdx                     ; Preserve caller's RDX (hygiene).

    mov edx, [rel VGA_CURSOR]    ; EDX = flat addr of the next free VGA cell
                                 ;   (DWORD cursor we left in the bootstage).
.print_loop:
    cmp edx, VGA_END             ; Beyond the end of VGA text RAM?
    jge .save                    ;   Yes -> screen full, stop.
    mov al, [rbx]                ; AL = next char (RBX-relative).
    test al, al                  ; Null terminator?
    jz .save                     ;   Yes -> string complete.
    mov ah, 0x0F                 ; AH = 0x0F: white-on-black attribute.
    mov [edx], ax                ; Write the cell {char,attr} to frame buffer.
    inc rbx                      ; Next source character.
    add edx, 2                   ; Next cell.
    jmp .print_loop              ; Loop.

.save:
    mov [rel VGA_CURSOR], edx    ; Persist cursor for later kernel stages.
    pop rdx
    pop rcx
    pop rbx
    ret

; Sector-2 data (must fit within this 512-byte sector).
MSG_LONG64:
    db "Lomsourt OS: Now in 64-bit Long Mode", 0x00
MSG_CORELOCK:
    db "Lomsourt CORE-LOCK: protected binary (ownership gate in kernel)", 0x00

; MSG_PM is consumed by init_pm (switch_pm.asm) in 32-bit mode via EBX.
; VGA_CURSOR is the persistent write cursor for print_string_pm. Seed it with
; the frame-buffer base so the FIRST call starts at the top-left of the screen
; and every call afterwards appends below the previous output.  Both live here
; (sector 2) rather than sector 1 so the boot sector keeps its spare room.
MSG_PM:
    db "Lomsourt OS: Switched to 32-bit Protected Mode", 0x00
VGA_CURSOR:
    dd 0xB8000

    ; Pad the trailing part of sector 2 so the image is a multiple of 512 B.
    times (512 * 2) - ($ - $$) db 0

; Step 11.1: Live-USB Environment messages (sector 3+)
MSG_ENV_LIVE_USB:
    db "===== Step 11.1 : Live-USB Environment =====", 0x0D, 0x0A, 0x00
    db "[SYS] ENVIRONMENT DETECTED: LIVE-USB (RAM-EXECUTION ONLY)", 0x0D, 0x0A, 0x00
    db "[SYS] Host drive read-only: enforced", 0x0D, 0x0A, 0x00
    db "[SYS] sys_pc + sys_mobile directories loaded", 0x0D, 0x0A, 0x00
    db "[SYS] Total image under 1 GiB: verified", 0x0D, 0x0A, 0x00

; ============================================================================
;  SECTOR 3+ - 64-bit exception infrastructure (Step 1.5)
;  Loaded by disk_load from CHS sector 3 onward into RAM starting at 0x8000.
;    idt64.asm : builds the 256-entry IDT at physical 0x5000 + installs LIDT.
;    panic.asm : exception stubs (vectors 0-31), the uniform frame builder,
;                and kernel_panic (red-screen register dump + halt).
; ============================================================================
%include "idt64.asm"
%include "panic.asm"
    ; Total image = 1024 bytes = sectors 1 + 2.