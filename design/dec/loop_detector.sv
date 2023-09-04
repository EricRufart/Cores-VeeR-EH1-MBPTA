module loop_detector #(
    parameter ENTRIES = 4
) (
    input  logic        clk,          // Top level clock
    input  logic        active_clk,   // Level 1 active clock
    input  logic        clk_override, // Override multiply clock enables
    input  logic        rst_l,        // Reset
    input  logic        scan_mode,    

    input  logic        inst_valid,
    input  logic        is_call,
    input  logic        dec_takenbr,
    input  logic [31:1] dec_takenbr_target,
    input  logic [31:1] last_pc,

    output logic        lock_cache, 
    output logic        lock_start 
);
    localparam PTR_SZ = $clog2(ENTRIES);

    typedef struct packed {
        logic [31:1] first, last;
    } loop_entry_t;

    loop_entry_t stack [ENTRIES-1:0];
    loop_entry_t new_entry, top_entry;

    logic [PTR_SZ-1:0] head_ff, head_adv, head_dec, tail_ff, tail_n;
    logic pc_after_last, pc_before_first;
    logic push, pop, adv_tail;
    logic empty;
    logic last_lock_ff;

    logic innermost_loop;

    // Read the stack
    assign top_entry = stack[head_dec];

    // Prepare an entry to be pushed
    assign new_entry.first = dec_takenbr_target;
    assign new_entry.last  = last_pc;

    // Check if the last PC was in the innermost loop
    assign pc_before_first = last_pc < top_entry.first;
    assign pc_after_last   = last_pc > top_entry.last;
    assign innermost_loop  = ~empty & ~(pc_before_first | pc_after_last);

    // Advance Head and tail
    assign head_adv = head_ff + PTR_SZ'(1);
    assign head_dec = head_ff - PTR_SZ'(1);
    assign tail_n   = tail_ff + PTR_SZ'(1);
    assign empty    = head_ff == tail_ff;
    
    //
    assign pop        = inst_valid & ~innermost_loop;
    // After a taken branch, we get a window with fluhed insn, keep the last lock state
    // until we get a valid PC
    assign lock_cache = push | ((/*inst_valid ? innermost_loop : */last_lock_ff) & ~empty & ~is_call);
    assign lock_start = push;

    // Conditional branches do not need the extra check, but JALs need it
    // We assume that calls to functions always break the loop
    assign push     = dec_takenbr & (dec_takenbr_target < last_pc) & ~is_call;
    assign adv_tail = push & ~pop & (head_adv == tail_ff);
   
    // Tail and Head flops
    // Function calls clear the stack
    rvdffsc  #(PTR_SZ)  headff (.*, .clk(active_clk), .clear(is_call), .en(push^pop), .din(push?head_adv:head_dec), .dout(head_ff) );
    rvdffsc  #(PTR_SZ)  tailff (.*, .clk(active_clk), .clear(is_call), .en(adv_tail), .din(tail_n),                 .dout(tail_ff) );
    rvdff    #(1)  lastff (.*, .clk(active_clk), .din(lock_cache), .dout(last_lock_ff) );

    // Stack flops
    for(genvar i = 0; i < ENTRIES; ++i) begin
        // Push/replace logic for each cell
        logic is_push    = push & ~pop & (head_ff  == PTR_SZ'(i));
        logic is_pushpop = push &  pop & (head_dec == PTR_SZ'(i));

        rvdffs  #($bits(loop_entry_t))  stackff (
            .clk  (active_clk),
            .en   (is_push | is_pushpop),
            .din  (new_entry ),
            .dout (stack[i]  ),
            .*
        );
    end


endmodule
