// ============================================================
// tt_um_traffic_ctrl — Tiny Tapeout top-level wrapper
// ============================================================
// Pinout  (SKY130 shuttle standard interface)
// -------------------------------------------
// ui_in[7:0]   — dedicated inputs
//   [0]  sensor_ns   vehicle sensor, N/S road
//   [1]  sensor_ew   vehicle sensor, E/W road
//   [2]  emrg_ns     emergency vehicle on N/S axis
//   [3]  emrg_ew     emergency vehicle on E/W axis
//   [7:4] reserved (tie low)
//
// uo_out[7:0]  — dedicated outputs
//   [1:0]  sig_ns[1:0]   N/S signal  (00=RED 01=GREEN 10=YELLOW)
//   [3:2]  sig_ew[1:0]   E/W signal
//   [4]    emrg_active   high during any emergency state
//   [7:5]  state_out     current FSM state (3 bits, for 7-seg debug)
//
// uio_in/out/oe[7:0] — bidirectional (unused; outputs tied low)
//
// Timing note
// -----------
// TT demo boards typically provide a 10 kHz clock via the RP2040.
// At 10 kHz with GREEN_TIME=30 the green phase is 3 ms — fine for
// demo. For realistic intersection timing swap parameters:
//   10 kHz  × GREEN_TIME=300   →  30 s green
//   10 kHz  × YELLOW_TIME=50   →   5 s yellow
// ============================================================

`default_nettype none

module tt_um_traffic_ctrl (
    input  wire [7:0] ui_in,    // dedicated inputs
    output wire [7:0] uo_out,   // dedicated outputs
    input  wire [7:0] uio_in,   // IOs: input path (unused)
    output wire [7:0] uio_out,  // IOs: output path (tie low)
    output wire [7:0] uio_oe,   // IOs: enable path  (tie low = input)
    input  wire       ena,      // design enable (active high)
    input  wire       clk,      // clock
    input  wire       rst_n     // reset, active low
);

    // Tie off bidirectional pins — not used in this design
    assign uio_out = 8'b0;
    assign uio_oe  = 8'b0;   // all bidirectional pins set as inputs

    // ---- unpack inputs ----
    wire sensor_ns = ui_in[0];
    wire sensor_ew = ui_in[1];
    wire emrg_ns   = ui_in[2];
    wire emrg_ew   = ui_in[3];

    // ---- internal signals ----
    wire [1:0] sig_ns;
    wire [1:0] sig_ew;
    wire       emrg_active;
    wire [2:0] state_out;

    // ---- instantiate FSM ----
    // Parameters tuned for a 10 kHz demo clock.
    // Multiply all time params by your clock frequency in Hz to get
    // real-world durations.
    traffic_fsm #(
        .GREEN_TIME  (30),   // cycles per green phase
        .YELLOW_TIME ( 5),   // cycles per yellow phase
        .HOLD_TIME   ( 3),   // all-red hold before emergency grant
        .EMRG_TIME   (20),   // cycles for emergency green window
        .MIN_REMAINING(5),   // extend green if ≤ this many cycles left
        .EXTEND_STEP (10),   // cycles to add per extension
        .MAX_GREEN   (60)    // hard ceiling on green duration
    ) u_fsm (
        .clk         (clk       ),
        .rst_n       (rst_n & ena),  // gate with ena: design goes idle when disabled
        .sensor_ns   (sensor_ns ),
        .sensor_ew   (sensor_ew ),
        .emrg_ns     (emrg_ns   ),
        .emrg_ew     (emrg_ew   ),
        .sig_ns      (sig_ns    ),
        .sig_ew      (sig_ew    ),
        .emrg_active (emrg_active),
        .state_out   (state_out )
    );

    // ---- pack outputs ----
    assign uo_out[1:0] = sig_ns;
    assign uo_out[3:2] = sig_ew;
    assign uo_out[4]   = emrg_active;
    assign uo_out[7:5] = state_out;

    wire _unused = &{uio_in, 1'b0};

endmodule
