// display hexadecimal digits to Kyoritsu OLED display
// (C) Oct 2016, izumi@ieee.org

`default_nettype none

module HEX2OLED_module
  (dbg1,dbg2,dbg3,dbg4,txd,clk,rstn);
   input  /* */ wire [31:0] dbg1,dbg2,dbg3,dbg4;
   output /* */ wire txd;
   input  /* */ wire clk,rstn;

   parameter PULSEW  = 5208; // 50MHz/9600bps=5208
   parameter PULSEW2 = 2604; // PULSEW/2

   reg [7:0] txch_r=0;
   reg       txchv_r=0;
   wire      txchw;

   defparam  uart0tx.PULSEW  =PULSEW;
   defparam  uart0tx.PULSEW2 =PULSEW2;
   UART_TX_module uart0tx(txch_r,txchv_r,txchw,txd,clk,rstn);

   function [7:0] bit4char;
      input [3:0] vec;
      case (vec)
        4'h0: bit4char=8'h30;
        4'h1: bit4char=8'h31;
        4'h2: bit4char=8'h32;
        4'h3: bit4char=8'h33;
        4'h4: bit4char=8'h34;
        4'h5: bit4char=8'h35;
        4'h6: bit4char=8'h36;
        4'h7: bit4char=8'h37;
        4'h8: bit4char=8'h38;
        4'h9: bit4char=8'h39;
        4'ha: bit4char=8'h41;
        4'hb: bit4char=8'h42;
        4'hc: bit4char=8'h43;
        4'hd: bit4char=8'h44;
        4'he: bit4char=8'h45;
        4'hf: bit4char=8'h46;
        default: bit4char=0;
      endcase
   endfunction

   reg [5:0]      count=0;
   reg [127:0]    shiftbuf=0;
   reg [15:0]     delay=0;

   always @(posedge clk)
     if (rstn==0) begin
        count<=0;
        txch_r<=0;
        txchv_r<=0;
        delay<=16'hffff;
     end else if (txchw==1) begin
        /* stay */
     end else if (count==38) begin
        txch_r<=8'h1b; //ESC
        txchv_r<=1;
        count<=count-1;
     end else if (count==37) begin
        txch_r<=8'h47; //G
        txchv_r<=1;
        count<=count-1;
     end else if (count==36) begin
        txch_r<=8'h40; //@
        txchv_r<=1;
        count<=count-1;
     end else if (count==35) begin
        txch_r<=8'h40; //@
        txchv_r<=1;
        count<=count-1;
     end else if (count==34) begin
        txchv_r<=0;
        if (delay==0) count<=count-1;
        delay<=delay-1;
     end else if (count==33) begin
        txchv_r<=0;
        if (delay==0) count<=count-1;
        delay<=delay-1;
     end else if (count<=32 && count>0) begin
        shiftbuf<={shiftbuf[123:0],4'b0};
        txch_r<=bit4char(shiftbuf[127:124]);
        txchv_r<=1;
        count<=count-1;
     end else if (count==0) begin
        shiftbuf<={dbg1,dbg2,dbg3,dbg4};
        txch_r<=0;
        txchv_r<=0;
        count<=38;
     end else begin
        count<=0;
        txch_r<=0;
        txchv_r<=0;
        delay<=16'hffff;
     end

endmodule

`default_nettype wire
