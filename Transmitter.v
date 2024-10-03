module BaudGenT(
    input wire         reset_n,           //  Active low reset.
    input wire         clock,             //  The System's main clock.
    input wire  [1:0]  baud_rate,         //  Baud Rate agreed upon by the Tx and Rx units.

    output reg         baud_clk           //  Clocking output for the other modules.
);

//  Internal declarations
reg [13 : 0] clock_ticks;
reg [13 : 0] final_value;

//  Encoding for the Baud Rates states
localparam BAUD24  = 2'b00,
           BAUD48  = 2'b01,
           BAUD96  = 2'b10,
           BAUD192 = 2'b11;

//  BaudRate 4-1 Mux
always @(*)
begin
    case (baud_rate)
        //  All these ratio ticks are calculated for 50MHz Clock,
        //  The values shall change with the change of the clock frequency.
        BAUD24 : final_value = 14'd10417;  //  ratio ticks for the 2400 BaudRate.
        BAUD48 : final_value = 14'd5208;   //  ratio ticks for the 4800 BaudRate.
        BAUD96 : final_value = 14'd2604;   //  ratio ticks for the 9600 BaudRate.
        BAUD192 : final_value = 14'd1302;  //  ratio ticks for the 19200 BaudRate.
        default: final_value = 14'd0;      //  The systems original Clock.
    endcase
end

//  Timer logic
always @(negedge reset_n, posedge clock)
begin
    if(!reset_n)
    begin
        clock_ticks <= 14'd0; 
        baud_clk    <= 1'b0; 
    end
    else
    begin
        if (clock_ticks == final_value)
        begin
            clock_ticks <= 14'd0; 
            baud_clk    <= ~baud_clk; 
        end 
        else
        begin
            clock_ticks <= clock_ticks + 1'd1;
            baud_clk    <= baud_clk;
        end
    end 
end

endmodule

Parity Generator
module Parity(
  input wire         reset_n,     //  Active low reset.
  input wire  [7:0]  data_in,     //  The data input from the InReg unit.
  input wire  [1:0]  parity_type, //  Parity type agreed upon by the Tx and Rx units.

  output reg         parity_bit   //  The parity bit output for the frame.
);

//  Encoding for the parity types
localparam ODD        = 2'b01,
           Even       = 2'b10;

always @(*)
begin
  if (!reset_n) parity_bit = 1'b1;
  else
  begin
    case (parity_type)
    ODD:     parity_bit = (^data_in)? 1'b0 : 1'b1;
    Even:    parity_bit = (^data_in)? 1'b1 : 1'b0; 
    default: parity_bit = 1'b1;
    endcase
  end
end
endmodule

PISO
module PISO(
    input wire           reset_n,            //  Active low reset.
    input wire           send,               //  An enable to start sending data.
    input wire           baud_clk,           //  Clocking signal from the BaudGen unit.
    input wire           parity_bit,         //  The parity bit from the Parity unit.
    input wire [7:0]     data_in,            //  The data input.
  
    output reg 	         data_tx, 	         //  Serial transmitter's data out
	output reg 	         active_flag,        //  high when Tx is transmitting, low when idle.
	output reg  	     done_flag 	         //  high when transmission is done, low when active.
);

//  Internal declarations
reg [3:0]   stop_count;
reg [10:0]  frame_r;
reg [10:0]  frame_man;
reg         next_state;
wire        count_full;

//  Encoding the states
localparam IDLE   = 1'b0,
           ACTIVE = 1'b1;

//  Frame generation
always @(posedge baud_clk, negedge reset_n) begin
    if (!reset_n)        frame_r <= {11{1'b1}};
    else if (next_state) frame_r <= frame_r;
    else                 frame_r <= {1'b1,parity_bit,data_in,1'b0};
end

// Counter logic
always @(posedge baud_clk, negedge reset_n) begin
    if (!reset_n || !next_state || count_full) stop_count <= 4'd0;
    else  stop_count <= stop_count + 4'd1;
end
assign count_full     = (stop_count == 4'd11);

//  Transmission logic FSM
always @(posedge baud_clk, negedge reset_n) begin
    if (!reset_n) next_state   <= IDLE;
	else
	begin
		if (!next_state) begin
            if (send) next_state   <= ACTIVE;
            else      next_state   <= IDLE;
        end
        else begin
            if (count_full) next_state   <= IDLE;
            else            next_state   <= ACTIVE;
        end
	end 
end

always @(*) begin
    if (reset_n && next_state && (stop_count != 4'd0)) begin
        data_tx      = frame_man[0];
        frame_man    = frame_man >> 1;
        active_flag  = 1'b1;
        done_flag    = 1'b0;
    end
    else begin
        data_tx      = 1'b1;
        frame_man    = frame_r;
        active_flag  = 1'b0;
        done_flag    = 1'b1;
    end
end

endmodule  

tx_unit
module TxUnit(
    input wire          reset_n,       //  Active low reset.
    input wire          send,          //  An enable to start sending data.
    input wire          clock,         //  The main system's clock.
    input wire  [1:0]   parity_type,   //  Parity type agreed upon by the Tx and Rx units.
    input wire  [1:0]   baud_rate,     //  Baud Rate agreed upon by the Tx and Rx units.
    input wire  [7:0]   data_in,       //  The data input.

    output        data_tx,             //  Serial transmitter's data out.
    output        active_flag,         //  high when Tx is transmitting, low when idle.
    output        done_flag            //  high when transmission is done, low when active.
);

//  Interconnections
wire parity_bit_w;
wire baud_clk_w;

//  Baud generator unit instantiation
BaudGenT Unit1(
    //  Inputs
    .reset_n(reset_n),
    .clock(clock),
    .baud_rate(baud_rate),
    
    //  Output
    .baud_clk(baud_clk_w)
);

//Parity unit instantiation 
Parity Unit2(
    //  Inputs
    .reset_n(reset_n),
    .data_in(data_in),
    .parity_type(parity_type),
    
    //  Output
    .parity_bit(parity_bit_w)
);

//  PISO shift register unit instantiation
PISO Unit3(
    //  Inputs
    .reset_n(reset_n),
    .send(send),
    .baud_clk(baud_clk_w),
    .data_in(data_in),
    .parity_bit(parity_bit_w),

    //  Outputs
    .data_tx(data_tx),
    .active_flag(active_flag),
    .done_flag(done_flag)
);

endmodule
