// ============================================================
// Adaptive Traffic Light Controller — FSM core
// ============================================================
// Intersection model: two roads, N/S and E/W.
// Each road has: RED, YELLOW, GREEN phases.
// Emergency override: freezes all lights → RED, then gives
// the emergency vehicle a dedicated GREEN on its approach axis.
//
// States
// ------
//   NS_GREEN   — N/S green,  E/W red
//   NS_YELLOW  — N/S yellow, E/W red   (transition guard)
//   EW_GREEN   — E/W green,  N/S red
//   EW_YELLOW  — E/W yellow, N/S red   (transition guard)
//   EMRG_HOLD  — all red, 2-cycle hold before grant
//   EMRG_NS    — emergency green on N/S axis
//   EMRG_EW    — emergency green on E/W axis
//
// Timing (all durations in clock cycles, parameterised)
// --------------------------------------------------------
//   GREEN_TIME   default 30  (extend/shorten by sensor)
//   YELLOW_TIME  default  5  (fixed — safety requirement)
//   HOLD_TIME    default  3  (all-red gap before EMRG grant)
//   EMRG_TIME    default 20  (emergency green window)
//
// Adaptive extension: if the active-road vehicle sensor stays
// asserted with < MIN_REMAINING cycles left, the green phase
// extends by one EXTEND_STEP (up to MAX_GREEN_TIME) so long
// as no emergency is pending.
// ============================================================

module traffic_fsm #(
    parameter CLK_HZ        = 10_000,  // clock frequency (for real use)

    // Phase durations in clock cycles
    parameter [5:0] GREEN_TIME    = 6'd30,
    parameter [5:0] YELLOW_TIME   = 6'd5,
    parameter [5:0] HOLD_TIME     = 6'd3,
    parameter [5:0] EMRG_TIME     = 6'd20,

    // Adaptive green extension
    parameter [5:0] MIN_REMAINING = 6'd5,       // extend if remaining <= this
    parameter [5:0] EXTEND_STEP   = 6'd10,
    parameter [5:0] MAX_GREEN     = 6'd60
)(
    input  wire clk,
    input  wire rst_n,

    // ---- sensor inputs ----
    input  wire sensor_ns,     // vehicle detected on N/S road
    input  wire sensor_ew,     // vehicle detected on E/W road
    input  wire emrg_ns,       // emergency vehicle approaching on N/S
    input  wire emrg_ew,       // emergency vehicle approaching on E/W

    // ---- outputs (2 bits per signal: {YELLOW, GREEN}; RED = 2'b00) ----
    // Encoding: 2'b00 = RED, 2'b01 = GREEN, 2'b10 = YELLOW
    output reg [1:0] sig_ns,   // N/S signal
    output reg [1:0] sig_ew,   // E/W signal

    // ---- status outputs ----
    output reg       emrg_active,   // high while any emergency state active
    output reg [2:0] state_out      // current FSM state (debug / 7-seg)
);

// ---------- state encoding ----------
localparam [2:0]
    NS_GREEN  = 3'd0,
    NS_YELLOW = 3'd1,
    EW_GREEN  = 3'd2,
    EW_YELLOW = 3'd3,
    EMRG_HOLD = 3'd4,
    EMRG_NS   = 3'd5,
    EMRG_EW   = 3'd6;

// ---------- signal encoding ----------
localparam [1:0] SIG_RED    = 2'b00;
localparam [1:0] SIG_GREEN  = 2'b01;
localparam [1:0] SIG_YELLOW = 2'b10;

// ---------- internal registers ----------
reg [2:0] state, next_state;
reg [5:0] timer;        // counts down to 0
reg [5:0] green_limit;  // current green ceiling (adaptive)
reg       emrg_pending; // latched emergency request

// ============================================================
// Sequential: state + timer update
// ============================================================

wire in_green_ns = (state == NS_GREEN);
wire in_green_ew = (state == EW_GREEN);
wire sensor_active = (in_green_ns && sensor_ns) || (in_green_ew && sensor_ew);
wire can_extend = (timer <= MIN_REMAINING) && (green_limit + EXTEND_STEP <= MAX_GREEN) && !emrg_pending && sensor_active;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state        <= NS_GREEN;
        timer        <= GREEN_TIME;
        green_limit  <= GREEN_TIME;
        emrg_pending <= 1'b0;
    end else begin
        // Latch any emergency request; cleared when we enter an EMRG state
        if (emrg_ns | emrg_ew)
            emrg_pending <= 1'b1;
        else if (state == EMRG_NS || state == EMRG_EW)
            emrg_pending <= 1'b0;
    
    
        if (timer == 6'd0) begin
            state       <= next_state;
            timer       <= timer_for(next_state);
            green_limit <= (next_state == NS_GREEN || next_state == EW_GREEN)
                       ? GREEN_TIME : green_limit;
        end else if (can_extend) begin
            green_limit <= green_limit + EXTEND_STEP;
            timer       <= timer + EXTEND_STEP;
        end else begin
            timer <= timer - 6'd1;
        end
    end
end

// ============================================================
// Function: initial timer value for a given state
// ============================================================
function [5:0] timer_for(input [2:0] s);
    case (s)
        NS_GREEN  : timer_for = GREEN_TIME;
        NS_YELLOW : timer_for = YELLOW_TIME;
        EW_GREEN  : timer_for = GREEN_TIME;
        EW_YELLOW : timer_for = YELLOW_TIME;
        EMRG_HOLD : timer_for = HOLD_TIME;
        EMRG_NS   : timer_for = EMRG_TIME;
        EMRG_EW   : timer_for = EMRG_TIME;
        default   : timer_for = GREEN_TIME;
    endcase
endfunction

// ============================================================
// Combinational: next-state logic
// Priority: emergency preemption > normal phase rotation
// ============================================================
always @(*) begin
    case (state)
        NS_GREEN: begin
            if (emrg_pending)
                next_state = EMRG_HOLD;
            else
                next_state = NS_YELLOW;
        end
        NS_YELLOW: begin
            if (emrg_pending)
                next_state = EMRG_HOLD;
            else
                next_state = EW_GREEN;
        end
        EW_GREEN: begin
            if (emrg_pending)
                next_state = EMRG_HOLD;
            else
                next_state = EW_YELLOW;
        end
        EW_YELLOW: begin
            if (emrg_pending)
                next_state = EMRG_HOLD;
            else
                next_state = NS_GREEN;
        end
        EMRG_HOLD: begin
            // Grant green to whichever axis has the emergency vehicle.
            // N/S takes priority if both axes signal simultaneously.
            if (emrg_ns)
                next_state = EMRG_NS;
            else if (emrg_ew)
                next_state = EMRG_EW;
            else
                next_state = NS_GREEN;  // spurious hold — resume normal
        end
        EMRG_NS: begin
            // After emergency window, return to normal via all-red hold
            next_state = (emrg_ew) ? EMRG_HOLD : NS_GREEN;
        end
        EMRG_EW: begin
            next_state = (emrg_ns) ? EMRG_HOLD : EW_GREEN;
        end
        default: next_state = NS_GREEN;
    endcase
end

// ============================================================
// Output logic (Moore machine — outputs depend only on state)
// ============================================================
always @(*) begin
    // Safe defaults: everything red
    sig_ns       = SIG_RED;
    sig_ew       = SIG_RED;
    emrg_active  = 1'b0;
    state_out    = state;

    case (state)
        NS_GREEN:  begin sig_ns = SIG_GREEN;  sig_ew = SIG_RED;    end
        NS_YELLOW: begin sig_ns = SIG_YELLOW; sig_ew = SIG_RED;    end
        EW_GREEN:  begin sig_ns = SIG_RED;    sig_ew = SIG_GREEN;  end
        EW_YELLOW: begin sig_ns = SIG_RED;    sig_ew = SIG_YELLOW; end
        EMRG_HOLD: begin sig_ns = SIG_RED;    sig_ew = SIG_RED;    emrg_active = 1'b1; end
        EMRG_NS:   begin sig_ns = SIG_GREEN;  sig_ew = SIG_RED;    emrg_active = 1'b1; end
        EMRG_EW:   begin sig_ns = SIG_RED;    sig_ew = SIG_GREEN;  emrg_active = 1'b1; end
        default:   begin sig_ns = SIG_RED;    sig_ew = SIG_RED;    end
    endcase
end

endmodule
