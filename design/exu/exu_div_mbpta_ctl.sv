

module exu_div_mbpta_ctl
	import swerv_types::*;
(
   input logic         clk,                       // Top level clock
   input logic         active_clk,                // Level 1 active clock
   input logic         rst_l,                     // Reset
   input logic         scan_mode,                 // Scan mode

   input logic         dec_tlu_fast_div_disable,  // Disable small number optimization

   input logic  [31:0] dividend,                  // Numerator
   input logic  [31:0] divisor,                   // Denominator

   input div_pkt_t     dp,                        // valid, sign, rem

   input logic         flush_lower,               // Flush pipeline


   output logic        valid_ff_e1,               // Valid E1 stage
   output logic        finish_early,              // Finish smallnum
   output logic        finish,                    // Finish smallnum or normal divide
   output logic        div_stall,                 // Divide is running

   output logic [31:0] out                        // Result
 );

	logic  	     finish_out;
	logic   	 finish_early_out;
	logic   	 div_stall_out;
	logic [ 5:0] counter;
	logic [ 5:0] counter_ff; // output of the counter
	logic 		 running;
	logic		 running_ff;
	logic [31:0] out_value;
	logic [31:0] out_value_ff;

	localparam NUMBERCYCLES = 6'h22; // 22 3F

	// launch the division and capture the finish value
	exu_div_ctl div_e1    (.*,
	                          .flush_lower   ( flush_lower        ),   // I
	                          .dp            ( dp                 ),   // I
	                          .dividend      ( dividend[31:0]     ),   // I
	                          .divisor       ( divisor[31:0]      ),   // I
	                          .valid_ff_e1   ( valid_ff_e1        ),   // O
	                          .div_stall     ( div_stall_out      ),   // O
	                          .finish_early  ( finish_early_out   ),   // O
	                          .finish        ( finish_out         ),   // O
	                          .out           ( out_value[31:0]    ));  // O

	
	// if we are running count
	assign counter = running_ff == 1'b1 ? counter_ff + 1 : 6'b000000;
	// update running If counter is 24 (36 iterations) and running set to 1 -> set running to 0 (Stop) else if is running continue
	assign running = counter == NUMBERCYCLES && running_ff ? 0'b0 : running_ff == 1'b1 ? 1'b1 : div_stall_out == 1'b1 ? 1'b1 : 1'b0;
	
	//assign running = counter == 6'h24 ? 0'b0 : running_ff == 1'b1 ? running_ff : 1'b1;
	//assign stall = div_stall_out && running_ff ? stall_ff : div_stall_out && !running_ff ? 1'b1 : 1'b0;
	//assign finish = finish_out;
	//assign finish_early = finish_early_out;
	assign finish = counter == NUMBERCYCLES - 1 && running_ff ? 1'b1 : 1'b0;
	assign out = counter == NUMBERCYCLES ? out_value : out_value_ff;
	assign div_stall = running;
	//assign div_stall = div_stall_out;

	rvdff  #(6)  counter_base_ff      (.*, .clk(active_clk), .din(counter),   .dout(counter_ff));
	rvdff  #(1)  running_base_ff      (.*, .clk(active_clk), .din(running),   .dout(running_ff));
	rvdff  #(32)  out_base_ff         (.*, .clk(active_clk), .din(out), .dout(out_value_ff));

endmodule // exu_div_mbpta_ctl