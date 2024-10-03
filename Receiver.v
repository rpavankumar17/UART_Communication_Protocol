module BaudGenR(
    input wire         reset_n,     //  Active low reset.
    input wire         clock,       //  The System's main clock.
    input wire  [1:0]  baud_rate,   //  Baud Rate agreed upon by the Tx and Rx units.

    output reg         baud_clk     //  Clocking output for the other modules.
);

//  Internal declarations
reg [9:0]  final_value;  //  Holds the number of ticks for each BaudRate.
reg [9:0]  clock_ticks;  //  Counts untill it equals final_value, Timer principle.

//  Encoding the different Baud Rates
localparam BAUD24      = 2'b00,
           BAUD48      = 2'b01,
           BAUD96      = 2'b10,
           BAUD192     = 2'b11;

//  BaudRate 4-1 Mux
always @(*) 
begin
    case (baud_rate)
      //  All these ratio ticks are calculated for 50MHz Clock,
      //  The values shall change with the change of the clock frequency.
      BAUD24: final_value  = 10'd651;     //  16 * 2400 BaudRate.
      BAUD48: final_value  = 10'd326;     //  16 * 4800 BaudRate.
      BAUD96: final_value  = 10'd163;     //  16 * 9600 BaudRate.
      BAUD192: final_value = 10'd81;      //  16 * 19200 BaudRate.
      default: final_value = 10'd163;     //  16 * 9600 BaudRate.
    endcase
end

//  Timer logic
always @(negedge reset_n, posedge clock) 
begin
  if(!reset_n) 
  begin
    clock_ticks   <= 10'd0;
    baud_clk      <= 1'b0;
  end
  else 
  begin
    if(clock_ticks == final_value)
    begin
      baud_clk      <= ~baud_clk;
      clock_ticks   <= 10'd0;
    end
    else 
    begin
      clock_ticks   <= clock_ticks + 1'd1;
      baud_clk      <= baud_clk;
    end
  end
end

endmodule

module SIPO(
    input  wire         reset_n,        //  Active low reset.
    input  wire         data_tx,        //  Serial Data recieved from the transmitter.
    input  wire         baud_clk,       //  The clocking input comes from the sampling unit.

    output wire         active_flag,    //  outputs logic 1 when data is in progress.
    output wire         recieved_flag,  //  outputs a signal enables the deframe unit. 
    output wire  [10:0] data_parll      //  outputs the 11-bit parallel frame.
);
//  Internal
reg [10:0] temp, data_parll_temp;
reg [3:0]  frame_counter, stop_count;
reg [1:0]  next_state;

//  Encoding the states of the reciever
localparam IDLE   = 2'b00,
           CENTER = 2'b01,
           FRAME  = 2'b11,
           GET    = 2'b10;

//  Recieving logic FSM
always @(posedge baud_clk, negedge reset_n) begin
  if (!reset_n) begin
    next_state    <= IDLE;
    stop_count    <= 4'd0;
    frame_counter <= 4'd0;
    temp          <= {11{1'b1}};
  end
  else begin
    case (next_state)
      IDLE : 
      begin
        temp          <= {11{1'b1}};
        stop_count    <= 4'd0;
        frame_counter <= 4'd0;
        if(~data_tx) begin
          next_state  <= CENTER;
        end
        else begin
          next_state  <= IDLE;
        end
      end

      CENTER : 
      begin
        if(stop_count == 4'd6) begin
          stop_count     <= 4'd0;
          next_state     <= GET;
        end
        else begin
          stop_count  <= stop_count + 4'b1;
          next_state  <= CENTER;
        end
      end

      FRAME :
      begin
        temp <= data_parll_temp;
        if(frame_counter == 4'd10) begin
          frame_counter <= 4'd0;
          next_state    <= IDLE;
        end
        else begin
          if(stop_count == 4'd14) begin
            frame_counter  <= frame_counter + 4'b1;
            stop_count     <= 4'd0; 
            next_state     <= GET;
          end
          else begin
            frame_counter  <= frame_counter;
            stop_count     <= stop_count + 4'b1;
            next_state     <= FRAME;
          end
        end
      end

      GET : begin 
        next_state     <= FRAME;
        temp           <= data_parll_temp;
      end
    endcase
  end
end

always @(*) begin
  case (next_state)
    IDLE, CENTER, FRAME: data_parll_temp  = temp;

    GET : begin
      data_parll_temp    = temp >> 1;
      data_parll_temp[10] = data_tx;
    end
  endcase
end

assign data_parll    = recieved_flag? data_parll_temp : {11{1'b1}};
assign recieved_flag = (frame_counter == 4'd10);
assign active_flag   = !recieved_flag;

endmodule

module DeFrame(
    input wire  [10:0]  data_parll,     //  Data frame passed from the sipo unit.
    input wire          recieved_flag,

    output reg          parity_bit,     //  The parity bit separated from the data frame.
    output reg          start_bit,      //  The Start bit separated from the data frame.
    output reg          stop_bit,       //  The Stop bit separated from the data frame.
    output reg          done_flag,      //  Indicates that the data is recieved and ready for another data packet.
    output reg  [7:0]   raw_data        //  The 8-bits data separated from the data frame.
);

//  Deframing 
always @(*) 
begin
  start_bit       = data_parll[0];
  raw_data[7:0]   = data_parll[8:1];
  parity_bit      = data_parll[9];
  stop_bit        = data_parll[10];
  done_flag       = recieved_flag;
end

endmodule

module ErrorCheck(
    input wire         reset_n,       //  Active low reset.
    input wire         recieved_flag, //  enable from the sipo unit for the flags.
    input wire         parity_bit,    //  The parity bit from the frame for comparison.
    input wire         start_bit,     //  The Start bit from the frame for comparison.
    input wire         stop_bit,      //  The Stop bit from the frame for comparison.
    input wire  [1:0]  parity_type,   //  Parity type agreed upon by the Tx and Rx units.
    input wire  [7:0]  raw_data,      //  The 8-bits data separated from the data frame.

    output wire [2:0]  error_flag     //  {stop_flag,start_flag,parity_flag}
);

//  Internal
reg error_parity;
reg parity_flag;
reg start_flag;
reg stop_flag;

//  Encoding for types of the parity
localparam ODD        = 2'b01,
           EVEN       = 2'b10;

//  Parity Check logic
always @(*) 
begin
  case (parity_type)
    ODD:     error_parity = (^raw_data)? 1'b0 : 1'b1;
    EVEN:    error_parity = (^raw_data)? 1'b1 : 1'b0;
    default: error_parity = 1'b1;
  endcase
end

// Error Check logic
always @(*) begin
  parity_flag  = (error_parity ^ parity_bit);
  start_flag   = (start_bit || 1'b0);
  stop_flag    = ~(stop_bit && 1'b1);
end

//  Output logic
assign error_flag = (reset_n && recieved_flag)? {stop_flag,start_flag,parity_flag} : 3'b0;

endmodule

module RxUnit(
    input wire         reset_n,      //  Active low reset.
    input wire         data_tx,      //  Serial data recieved from the transmitter.
    input wire         clock,        //  The System's main clock.
    input wire  [1:0]  parity_type,  //  Parity type agreed upon by the Tx and Rx units.
    input wire  [1:0]  baud_rate,    //  Baud Rate agreed upon by the Tx and Rx units.

    output             active_flag,
    //  outputs logic 1 when data is in progress.
    output             done_flag,
    //  Outputs logic 1 when data is recieved
    output      [2:0]  error_flag,
    //  Consits of three bits, each bit is a flag for an error
    //  error_flag[0] ParityError flag, error_flag[1] StartError flag,
    //  error_flag[2] StopError flag.
    output      [7:0]  data_out
    //  The 8-bits data separated from the frame.
);

//  Intermediate wires
wire baud_clk_w;          //  The clocking signal from the baud generator.
wire [10:0] data_parll_w; //  data_out parallel comes from the SIPO unit.
wire recieved_flag_w;     //  works as an enable for deframe unit.
wire def_par_bit_w;       //  The Parity bit from the Deframe unit to the ErrorCheck unit.
wire def_strt_bit_w;      //  The Start bit from the Deframe unit to the ErrorCheck unit.
wire def_stp_bit_w;       //  The Stop bit from the Deframe unit to the ErrorCheck unit.

//  clocking Unit Instance
BaudGenR Unit1(
    //  Inputs
    .reset_n(reset_n),
    .clock(clock),
    .baud_rate(baud_rate),

    //  Output
    .baud_clk(baud_clk_w)
);

//  Shift Register Unit Instance
SIPO Unit2(
    //  Inputs
    .reset_n(reset_n),
    .data_tx(data_tx),
    .baud_clk(baud_clk_w),

    //  Outputs
    .active_flag(active_flag),
    .recieved_flag(recieved_flag_w),
    .data_parll(data_parll_w)
);

//  DeFramer Unit Instance
DeFrame Unit3(
    //  Inputs
    .data_parll(data_parll_w),
    .recieved_flag(recieved_flag_w),
    
    //  Outputs
    .parity_bit(def_par_bit_w),
    .start_bit(def_strt_bit_w),
    .stop_bit(def_stp_bit_w),
    .done_flag(done_flag),
    .raw_data(data_out)
);

//  Error Checking Unit Instance
ErrorCheck Unit4(
    //  Inputs
    .reset_n(reset_n),
    .recieved_flag(done_flag),
    .parity_bit(def_par_bit_w),
    .start_bit(def_strt_bit_w),
    .stop_bit(def_stp_bit_w),
    .parity_type(parity_type),
    .raw_data(data_out),

    //  Output
    .error_flag(error_flag)
);

endmodul
