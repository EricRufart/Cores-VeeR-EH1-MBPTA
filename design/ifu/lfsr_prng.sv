`default_nettype none
//-----------------------------------------------------
// Project Name : a.out
// Function     : Generate random number
// Description  : Generate a random number and send by the output

//***Headers***
//***Module***
module lfsr_prng
    import swerv_types::*;
    (
        input logic  clk ,
	input logic rst_l,
        output logic [2 : 0] output_number_o 
    );

    logic [167:0] lfsr;
    logic [2:0] xnor_o;
    logic [3:0] check_0;
    logic [3:0] check_1;
    logic [3:0] check_2;

    initial begin
	    lfsr = 168'b111000110000011101000110000110100001010011100101111011001100111010000000100001100000110101000000011010010110001111001101000011000101110000010001100000000000101110000011;
    end


    assign xnor_o[2] = ~(^{lfsr[167],lfsr[135],lfsr[103],lfsr[71]});
    assign xnor_o[1] = ~(^{lfsr[166],lfsr[134],lfsr[102],lfsr[70]});
    assign xnor_o[0] = ~(^{lfsr[165],lfsr[133],lfsr[101],lfsr[69]});
    assign check_2 = {lfsr[167],lfsr[135],lfsr[103],lfsr[71]};
    assign check_1 = {lfsr[166],lfsr[134],lfsr[102],lfsr[70]};
    assign check_0 = {lfsr[165],lfsr[133],lfsr[101],lfsr[69]};
    assign output_number_o = xnor_o;
    rvdff #(168) random_seed_ff (.*, .clk(clk), .din({lfsr[164:0],xnor_o}), .dout(lfsr));


    //always @(posedge clk) begin
	    //output_number_o<=xnor_o;
	    //if(strobe)begin
	//	    lfsr<=lfsr;

	  //  end else begin
//		    lfsr<={lfsr[164:0],xnor_o};
	    //end
  //  end

endmodule
