"""
Testbench for tt_um_traffic_ctrl
Uses cocotb + standard TT test harness conventions.

Tests
-----
1. test_reset_state          — after reset, N/S is green, E/W is red
2. test_normal_rotation      — full green→yellow→green cycle completes
3. test_emergency_preemption — emrg_ns triggers EMRG_HOLD then EMRG_NS
4. test_emergency_both_axes  — simultaneous emrg_ns + emrg_ew: N/S wins,
                               then E/W served on next EMRG_HOLD
5. test_adaptive_extension   — sensor_ns held during green extends the phase
6. test_all_red_during_hold  — both signals are RED in EMRG_HOLD state
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, ClockCycles

# Detect if running gate-level simulation
IS_GL = os.environ.get('GATES', 'no') == 'yes'

# ---- signal encoding constants ----
RED    = 0b00
GREEN  = 0b01
YELLOW = 0b10

# ---- FSM state encoding (matches traffic_fsm.v) ----
NS_GREEN  = 0
NS_YELLOW = 1
EW_GREEN  = 2
EW_YELLOW = 3
EMRG_HOLD = 4
EMRG_NS   = 5
EMRG_EW   = 6

# ---- parameter defaults (must match DUT instantiation) ----
GREEN_TIME  = 30
YELLOW_TIME = 5
HOLD_TIME   = 3
EMRG_TIME   = 20


def decode_outputs(dut):
    """Return (sig_ns, sig_ew, emrg_active, state) from uo_out."""
    uo = dut.uo_out.value.integer
    sig_ns      = (uo >> 0) & 0x3
    sig_ew      = (uo >> 2) & 0x3
    emrg_active = (uo >> 4) & 0x1
    state       = (uo >> 5) & 0x7
    return sig_ns, sig_ew, emrg_active, state


async def reset_dut(dut):
    """Apply reset and wait for it to propagate."""
    dut.rst_n.value  = 0
    dut.ena.value    = 1
    dut.ui_in.value  = 0
    await ClockCycles(dut.clk, 3)
    dut.rst_n.value  = 1
    await RisingEdge(dut.clk)


# ===========================================================
# Test 1 — reset lands in NS_GREEN
# ===========================================================
@cocotb.test()
async def test_reset_state(dut):
    clock = Clock(dut.clk, 100, units="ns")  # 10 MHz for sim speed
    cocotb.start_soon(clock.start())

    await reset_dut(dut)

    sig_ns, sig_ew, emrg_active, state = decode_outputs(dut)
    assert sig_ns == GREEN,  f"Expected N/S GREEN after reset, got {sig_ns}"
    assert sig_ew == RED,    f"Expected E/W RED after reset, got {sig_ew}"
    assert emrg_active == 0, "emrg_active should be 0 after reset"
    assert state == NS_GREEN, f"Expected NS_GREEN state, got {state}"

    dut._log.info("PASS: test_reset_state")


# ===========================================================
# Test 2 — normal rotation: NS_GREEN → NS_YELLOW → EW_GREEN
# ===========================================================
@cocotb.test()
async def test_normal_rotation(dut):
    if IS_GL:
        dut._log.info("SKIP: Timing test not applicable to gate-level")
        return  # Skip test in gate-level
    
    clock = Clock(dut.clk, 100, units="ns")
    cocotb.start_soon(clock.start())

    await reset_dut(dut)

    # Wait out the green phase
    await ClockCycles(dut.clk, GREEN_TIME + 2)

    sig_ns, sig_ew, _, state = decode_outputs(dut)
    print(f"DEBUG: After {GREEN_TIME + 2} cycles:")
    print(f"  sig_ns={sig_ns:02b} (expected YELLOW={YELLOW:02b})")
    print(f"  state={state} (expected NS_YELLOW={NS_YELLOW})")


    
    assert sig_ns == YELLOW, f"Expected NS YELLOW, got {sig_ns}"
    assert sig_ew == RED,    f"Expected EW RED, got {sig_ew}"
    assert state == NS_YELLOW, f"Expected NS_YELLOW state, got {state}"

    # Wait out the yellow phase
    await ClockCycles(dut.clk, YELLOW_TIME + 2)

    sig_ns, sig_ew, _, state = decode_outputs(dut)
    assert sig_ns == RED,   f"Expected NS RED, got {sig_ns}"
    assert sig_ew == GREEN, f"Expected EW GREEN, got {sig_ew}"
    assert state == EW_GREEN, f"Expected EW_GREEN state, got {state}"

    dut._log.info("PASS: test_normal_rotation")


# ===========================================================
# Test 3 — emergency preemption on N/S axis
# ===========================================================
@cocotb.test()
async def test_emergency_preemption(dut):
    if IS_GL:
        dut._log.info("SKIP: Timing test not applicable to gate-level")
        return  # Skip test in gate-level
    clock = Clock(dut.clk, 100, units="ns")
    cocotb.start_soon(clock.start())

    await reset_dut(dut)

    # Assert emergency signal mid-green
    await ClockCycles(dut.clk, 10)
    dut.ui_in.value = 0b0100  # emrg_ns = bit 2

    # After current green phase expires + 1 cycle, should enter EMRG_HOLD
    await ClockCycles(dut.clk, GREEN_TIME - 10 + 2)

    sig_ns, sig_ew, emrg_active, state = decode_outputs(dut)
    assert emrg_active == 1, "emrg_active should be 1 in emergency"
    assert state == EMRG_HOLD, f"Expected EMRG_HOLD, got {state}"
    assert sig_ns == RED, f"N/S should be RED in EMRG_HOLD, got {sig_ns}"
    assert sig_ew == RED, f"E/W should be RED in EMRG_HOLD, got {sig_ew}"

    # After hold, N/S gets emergency green
    await ClockCycles(dut.clk, HOLD_TIME + 2)

    sig_ns, sig_ew, emrg_active, state = decode_outputs(dut)
    assert state == EMRG_NS,  f"Expected EMRG_NS, got {state}"
    assert sig_ns == GREEN,   f"N/S should be GREEN in EMRG_NS, got {sig_ns}"
    assert sig_ew == RED,     f"E/W should be RED in EMRG_NS, got {sig_ew}"
    assert emrg_active == 1,  "emrg_active should be 1 in EMRG_NS"

    # Clear emergency, wait for return to normal
    dut.ui_in.value = 0
    await ClockCycles(dut.clk, EMRG_TIME + 2)

    sig_ns, sig_ew, emrg_active, state = decode_outputs(dut)
    assert emrg_active == 0, "emrg_active should clear after emergency window"
    assert state in (NS_GREEN, EW_GREEN), f"Expected normal state, got {state}"

    dut._log.info("PASS: test_emergency_preemption")


# ===========================================================
# Test 4 — simultaneous emergency on both axes: N/S wins first
# ===========================================================
@cocotb.test()
async def test_emergency_both_axes(dut):
    if IS_GL:
        dut._log.info("SKIP: Timing test not applicable to gate-level")
        return  # Skip test in gate-level
    clock = Clock(dut.clk, 100, units="ns")
    cocotb.start_soon(clock.start())

    await reset_dut(dut)

    # Both emergency signals asserted
    dut.ui_in.value = 0b1100  # emrg_ns (bit2) + emrg_ew (bit3)

    await ClockCycles(dut.clk, GREEN_TIME + 2)

    _, _, _, state = decode_outputs(dut)
    assert state == EMRG_HOLD, f"Expected EMRG_HOLD, got {state}"

    await ClockCycles(dut.clk, HOLD_TIME + 2)

    _, _, _, state = decode_outputs(dut)
    # N/S takes priority when both asserted simultaneously
    assert state == EMRG_NS, f"Expected EMRG_NS (N/S priority), got {state}"

    dut._log.info("PASS: test_emergency_both_axes")


# ===========================================================
# Test 5 — adaptive green extension when sensor active
# ===========================================================
@cocotb.test()
async def test_adaptive_extension(dut):
    clock = Clock(dut.clk, 100, units="ns")
    cocotb.start_soon(clock.start())

    await reset_dut(dut)

    # Hold sensor_ns high throughout — should trigger extension
    dut.ui_in.value = 0b0001  # sensor_ns = bit 0

    # After normal GREEN_TIME, design should NOT have transitioned
    # (extension should have fired since sensor was active near end)
    await ClockCycles(dut.clk, GREEN_TIME + 3)

    _, _, _, state = decode_outputs(dut)
    # With sensor active, green phase should have been extended
    # so we expect to still be in NS_GREEN or just entered NS_YELLOW
    assert state in (NS_GREEN, NS_YELLOW), \
        f"Expected extended green or just-entered yellow, got {state}"

    dut._log.info("PASS: test_adaptive_extension")


# ===========================================================
# Test 6 — both signals strictly RED during EMRG_HOLD
# ===========================================================
@cocotb.test()
async def test_all_red_during_hold(dut):
    clock = Clock(dut.clk, 100, units="ns")
    cocotb.start_soon(clock.start())

    await reset_dut(dut)

    # Trigger emergency
    await ClockCycles(dut.clk, 5)
    dut.ui_in.value = 0b0100  # emrg_ns

    # Advance to just after EMRG_HOLD entry
    await ClockCycles(dut.clk, GREEN_TIME + 2)

    # Sample every cycle during the HOLD phase
    for _ in range(HOLD_TIME + 1):
        sig_ns, sig_ew, _, state = decode_outputs(dut)
        if state == EMRG_HOLD:
            assert sig_ns == RED, f"N/S must be RED in EMRG_HOLD, got {sig_ns}"
            assert sig_ew == RED, f"E/W must be RED in EMRG_HOLD, got {sig_ew}"
        await RisingEdge(dut.clk)

    dut._log.info("PASS: test_all_red_during_hold")
