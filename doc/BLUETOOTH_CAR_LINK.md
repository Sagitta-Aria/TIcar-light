# Bluetooth car-to-car link

This project supports one HC-05 on each car. Both cars use the same source
tree, but they must be built with different Bluetooth roles.

## Verified module pair

| Car role | HC-05 address | Persistent module role |
| --- | --- | --- |
| Master car | `00:25:06:01:0D:D3` | `ROLE=1` |
| Slave car | `98:DA:50:03:01:77` | `ROLE=0` |

The master module is configured with `CMODE=0` and
`BIND=98da,50,030177`. Both modules use PIN `1234` and transparent UART data
mode without flow control. The pair was previously verified at 9600 bit/s
after a cold restart with 50 frames in each direction and no missing frames.

The firmware data-mode target is now 115200 bit/s, 8-N-1. On 2026-07-27 the
slave module at `98da:50:030177` was changed and read back as
`+UART=115200,0,0`. The master module was not present on a USB serial port
during that operation and must also receive `AT+UART=115200,0,0` before the
two modules can exchange data with the new firmware. Full AT mode itself
remains fixed at 38400 bit/s.

The HC-05 stores `ROLE`, `BIND`, PIN, and UART settings in the module. The MCU
configuration mirrors both addresses for runtime identity checks; it does not
replace the module's persistent binding.

## Role-specific builds

```powershell
Set-Location D:\Ti\m0-light-rtos
.\tools\build_ccs.ps1 -Profile Gmr -Board Tianmeng -BluetoothRole Master -Clean
.\tools\build_ccs.ps1 -Profile Gmr -Board Dimeng -BluetoothRole Slave -Clean
```

For the current Tianmeng master combination with board JY61 and UART0 logging
disabled (external M0 attitude remains enabled and Gmr has no K230), build:

```powershell
.\tools\build_ccs.ps1 -Profile Gmr -Board Tianmeng -BluetoothRole Master -Jy61 Disabled -Log Disabled -Clean
```

The outputs are kept separate:

```text
Debug\profile-gmr-tianmeng-bt-master\m0-light-rtos.out
Debug\profile-gmr-dimeng-bt-slave\m0-light-rtos.out
```

`-BluetoothRole Disabled` is the default and preserves the previous project
behavior. `-Board Tianmeng` is the board default. Full Profile can use
Bluetooth only on Tianmeng because Full reserves UART3 PB2/PB3 for K230.

## Runtime identity check

Each node transmits a binary `HELLO` frame once per second. The payload carries
the sender role, sender HC-05 address, and expected peer HC-05 address. A link
becomes authenticated only when all of these fields match the compile-time
pair. It returns to the waiting state after 3.5 seconds without a valid peer
frame.

The transparent UART protocol uses:

```text
A5 5A | version | type | sequence | source | target | length | payload | CRC16
```

CRC16 is CRC-CCITT with polynomial `0x1021` and initial value `0xFFFF`.
Supported frame types are `HELLO`, `HEARTBEAT`, and `SIGNAL`. A signal contains
a 16-bit signal ID, signed 32-bit value, and 8-bit flags field.

Task-context application APIs are declared in `app/bluetooth_service.h`:

```c
BluetoothService_SendSignal(signalId, value, flags);
BluetoothService_TakeSignal(&signal);
BluetoothService_IsConnected();
```

The service uses fixed-size static FreeRTOS queues. UART receive parsing and
frame generation run in the existing 5 ms Comm task, not in an interrupt.

## Gmr Task 6: record and replay

Both cars must enter `Task 6 BT Replay`. Enter the Dimeng slave first when
possible, then enter the Tianmeng master. The master remains stopped until the
authenticated link is up and the slave has completed the `READY`, `START`,
`START_ACK` handshake.

The Task 6 signal sequence is:

```text
slave READY
master START(session)
slave START_ACK(session)
master ENCODER_DELTA(left count delta, right count delta), sampled every 50 ms
master OVER(total sample count)
slave OVER_ACK(total sample count)
slave replays cumulative distance targets without gray input
slave DONE(total sample count)
```

The master uses `MotorNoYaw_Start()` and treats four completed right-angle
turns as one lap. Every 50 ms it subtracts the previous left and right total
encoder counts from the current totals. Each non-zero `ENCODER_DELTA` packs
the two signed 16-bit count deltas into one 32-bit value. The flags byte
carries the sample index modulo 256 so the slave can reject missing,
duplicated, or out-of-order samples. These are measured distances, not PWM or
speed commands.

The slave statically allocates 2048 packed samples (8192 bytes), equal to at
least about 102.4 seconds of continuous motion at 50 ms sampling; stationary
windows are omitted. It does not start the chassis until `OVER` matches the
received count. During replay it adds each delta to cumulative left and right
position targets. A 10 ms position outer loop converts the remaining encoder
error to a bounded `count/20ms` target for the slave car's own encoder PI
loop. A sample completes only when both wheel errors enter the configured
tolerance, so replay can take longer than the recording when the slave needs
time to catch up. The OLED shows the current left/right position error as
`PosE`.

Task 6 application protocol version 2 is not compatible with the previous
speed-sample version; both cars must be reflashed together. Link loss, a
queue failure, sample-order failure, malformed `OVER`, distance overflow,
segment timeout, or buffer overflow stops the chassis and changes Task 6 to
the error state.

## Board-specific transport and wiring

Both board variants use 115200 bit/s, 8-N-1, without flow control. Tianmeng has
UART2 available for Bluetooth. Dimeng exposes the proven HC-05 route on UART3,
so a `Gmr/Dimeng` Bluetooth build transfers UART3 ownership from the external
M0 attitude link to Bluetooth.

| Tianmeng 64P | Header | HC-05 | Direction |
| --- | --- | --- | --- |
| PB15 / UART2 TX | U21-11 | RXD | MCU to HC-05 |
| PB16 / UART2 RX | U21-13 | TXD | HC-05 to MCU |
| GND | U21-21 or another GND | GND | Common ground |
| EXT_3V3 | U21-22 | VCC only when supported | Module power |

| Dimeng 48P | Header | HC-05 | Direction |
| --- | --- | --- | --- |
| PB2 / UART3 TX | H3-12 | RXD | MCU to HC-05 |
| PB3 / UART3 RX | H3-13 | TXD | HC-05 to MCU |
| GND | H3-20 | GND | Common ground |
| 3.3V | H3-19 | VCC only when supported | Module power |

UART signals are 3.3 V. Follow the HC-05 carrier-board VCC marking; do not put
a 5 V UART signal into PB16 or PB3. Keep HC-05 `KEY` low for normal transparent data
mode. `STATE` is not required because the authenticated HELLO frames provide
link state. Do not reuse PB20 for `STATE`; PB20 is the right encoder B input.

`hardware/bluetooth_uart.c` owns the selected UART power, pin mux, RX/TX FIFO
interrupts, and byte bridge. CRC parsing and identity checks remain in the
5 ms Comm task. The Dimeng slave build cannot receive external-M0 yaw on Task5
because HC-05 owns UART3. Board JY61/UART1 is unchanged.

## One-time HC-05 provisioning reference

The verified master settings are:

```text
AT+ROLE=1
AT+CMODE=0
AT+PSWD=1234
AT+BIND=98da,50,030177
AT+UART=115200,0,0
AT+INIT
AT+PAIR=98da,50,030177,20
AT+LINK=98da,50,030177
```

The verified slave settings are:

```text
AT+ROLE=0
AT+CMODE=0
AT+PSWD=1234
AT+UART=115200,0,0
```

Do not run provisioning on every boot. Normal transparent-data operation must
start with HC-05 `KEY` low.

## Verification

Run the wire-format vectors without hardware:

```powershell
.\tools\test_bluetooth_protocol.ps1
.\tools\test_gmr_distance_replay.ps1
```

Then build the disabled, master, and slave variants. Build scripts only compile
and link; they do not download firmware.

Build and safely download a Tianmeng Gmr master through XDS110 with:

```powershell
.\tools\flash_xds110_safe.ps1 -Profile Gmr -Board Tianmeng -BluetoothRole Master
```

To keep the external-M0 attitude link while disabling board JY61 and UART0
logging on that master, use:

```powershell
.\tools\flash_xds110_safe.ps1 -Profile Gmr -Board Tianmeng -BluetoothRole Master -Jy61 Disabled -Log Disabled
```

Build and safely download the Dimeng Gmr slave with:

```powershell
.\tools\flash_xds110_safe.ps1 -Profile Gmr -Board Dimeng -BluetoothRole Slave
```
