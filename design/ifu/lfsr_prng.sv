//********************************************************************************
// SPDX-License-Identifier: Apache-2.0
// Copyright 2019 Western Digital Corporation or its affiliates.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//********************************************************************************

//********************************************************************************
// Function: LFSR Pseudo-random number generator (64-bit, variable output size)
//********************************************************************************
//-----------------------------------------------------

module lfsr_prng
    import veer_types::*;
    # (
        parameter SIZE = 3
    ) (
        input  logic            clk ,
        input  logic            rst_l,
        input  logic [31:0]     seed_i,
        output logic [SIZE-1:0] output_number_o
    );

    // 64-bit XNOR LFSR (0 is a valid state)
    // Taps from: https://docs.xilinx.com/v/u/en-US/xapp052
    // taps = 64,63,61,60
    logic [63:0] lfsr, lfsr_next;
    logic newbit;
/*`ifndef SYNTHESIS
	 initial begin
			lfsr = seed_i;
	 end
`endif*/
    always_ff @(posedge clk or negedge rst_l) begin
        if (rst_l == 0)
          lfsr <= {seed_i[8:2], seed_i[31:17], seed_i[12:3] ,seed_i[31:0]};
        else
          lfsr <= lfsr_next;
    end

    assign newbit          = ~(lfsr[63] ^ lfsr[62] ^ lfsr[61] ^ lfsr[60]);
    assign lfsr_next       =  {lfsr[62:0], newbit};
    assign output_number_o = lfsr[SIZE-1:0];

endmodule
