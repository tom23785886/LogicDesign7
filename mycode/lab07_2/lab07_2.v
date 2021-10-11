`define INITIAL 2'b00
`define RTOL 2'b01
`define UTOD 2'b10
`define SPLIT 2'b11
module lab07_2(input wire clk,input wire rst,input wire shift,input wire split,
            output reg[3:0]vgaRed,output reg[3:0]vgaGreen,output reg[3:0]vgaBlue,output hsync,output vsync);

        wire [11:0] data;
    wire clk_25MHz;
    wire clk_22;
    wire clk21;
    wire clk20;
    wire clk19;
    wire clk23;
    wire clk22;
    wire [16:0] pixel_addr;
    wire [11:0] pixel;
    wire valid;
    wire [9:0] h_cnt; //640
    wire [9:0] v_cnt;  //480
    reg [9:0]rightbd;
    reg [9:0]nextrightbd;
    reg [9:0]midbd;
    reg [9:0]nextmidbd;
    reg [9:0]leftupbd;
    reg [9:0]nextleftupbd;
    reg [9:0]rightupbd;
    reg [9:0]nextrightupbd;
    reg [9:0]rightdwbd;
    reg [9:0]nextrightdwbd;
    reg [1:0]state;
    reg [1:0]nextstate;
    reg [8:0]dwbd;
    reg [8:0]nextdwbd;
    myclk_div #(23)clkmy4 (.clk(clk),.clk_div(clk23));
    myclk_div #(22)clkmy5 (.clk(clk),.clk_div(clk22));
    myclk_div #(21)clkmy3 (.clk(clk),.clk_div(clk21));
    myclk_div #(20)clkmy (.clk(clk),.clk_div(clk20));
    myclk_div #(19)clkmy2 (.clk(clk),.clk_div(clk19));
  always@(*)
    begin
        if(!valid) 
        begin 
            {vgaRed,vgaGreen,vgaBlue}=12'h0; 
            nextrightbd=rightbd; 
            /*nextleftupbd=leftupbd;
            nextrightdwbd=rightdwbd;*/
            nextstate=state;
            nextdwbd=dwbd;
        end
        else 
            begin
                if(rst)
                    begin
                        {vgaRed,vgaGreen,vgaBlue}=pixel; 
                        nextrightbd=640;
                        nextdwbd=0;
                        nextmidbd=320;
                        nextleftupbd=240;
                        nextrightupbd=320;
                        nextrightdwbd=240;
                        nextstate=`INITIAL;
                    end
                case(state)
                    `INITIAL:
                        begin
                            nextdwbd=dwbd;
                            nextmidbd=320;
                            nextleftupbd=240;
                            nextrightupbd=320;
                            nextrightdwbd=240;
                            if(shift)
                                begin
                                    {vgaRed,vgaGreen,vgaBlue}=pixel;
                                    nextrightbd=640;
                                    nextstate=`RTOL;
                                end
                            else if(split)
                                begin
                                    {vgaRed,vgaGreen,vgaBlue}=pixel;
                                    nextrightbd=640;
                                    nextstate=`SPLIT;
                                end
                            else
                                begin
                                    {vgaRed,vgaGreen,vgaBlue}=pixel; 
                                    nextrightbd=640;
                                    nextstate=`INITIAL;
                                end                            
                        end
                    `RTOL:
                        begin
                            nextmidbd=midbd;
                            nextleftupbd=leftupbd;
                            nextrightupbd=nextrightupbd;
                            nextrightdwbd=rightdwbd;
                            if(rightbd<=0)
                                begin
                                    {vgaRed,vgaGreen,vgaBlue}=12'h0; 
                                    nextrightbd=640;
                                    nextdwbd=0;
                                    nextstate=`UTOD;
                                end
                            else
                                begin
                                    
                                    nextdwbd=dwbd;
                                    if(h_cnt<rightbd)
                                        begin
                                            {vgaRed,vgaGreen,vgaBlue}=pixel; 
                                            nextrightbd=rightbd-1;
                                            nextstate=`RTOL;
                                        end
                                    else
                                        begin
                                            {vgaRed,vgaGreen,vgaBlue}=12'h0; 
                                            nextrightbd=rightbd-1;
                                            nextstate=`RTOL;
                                        end
                                end
                        end
                    `UTOD:
                        begin
                            nextmidbd=midbd;
                            nextrightbd=rightbd;
                            nextleftupbd=leftupbd;
                            nextrightupbd=rightupbd;
                            nextrightdwbd=rightdwbd;
                            if(dwbd>=480)
                                begin
                                    {vgaRed,vgaGreen,vgaBlue}=pixel;
                                    nextstate=`INITIAL;
                                    nextdwbd=0;
                                end
                            else
                                begin
                                    if(v_cnt<dwbd)  
                                        begin
                                            {vgaRed,vgaGreen,vgaBlue}=pixel; 
                                            nextdwbd=dwbd+1;
                                            nextstate=`UTOD;
                                        end
                                    else
                                        begin
                                            {vgaRed,vgaGreen,vgaBlue}=12'h0; 
                                            nextdwbd=dwbd+1;
                                            nextstate=`UTOD;
                                        end
                                end
                        end
                    `SPLIT:
                        begin
                            nextrightbd=rightbd;
                            nextdwbd=dwbd;
                            if(midbd<=1)
                                begin
                                    {vgaRed,vgaGreen,vgaBlue}=12'h0;
                                    nextstate=`INITIAL;
                                    nextmidbd=midbd;
                                    nextleftupbd=leftupbd;
                                    nextrightupbd=rightupbd;
                                    nextrightdwbd=rightdwbd;
                                end
                            else
                                begin
                                    if(h_cnt<=midbd&&v_cnt>240)
                                        begin
                                            {vgaRed,vgaGreen,vgaBlue}=pixel;
                                            nextstate=`SPLIT;
                                            nextmidbd=midbd-1;
                                            nextrightupbd=rightupbd+1;
                                            nextrightdwbd=rightdwbd+1;
                                            if(leftupbd>0) begin nextleftupbd=leftupbd-1; end
                                            else begin nextleftupbd=leftupbd; end
                                        end
                                    else if(h_cnt>midbd&&h_cnt<=320&&v_cnt>240)
                                        begin
                                            {vgaRed,vgaGreen,vgaBlue}=12'h0;
                                            nextstate=`SPLIT;
                                            nextmidbd=midbd-1;
                                            nextrightupbd=rightupbd+1;
                                            nextrightdwbd=rightdwbd+1;
                                            if(leftupbd>0) begin nextleftupbd=leftupbd-1; end
                                            else begin nextleftupbd=leftupbd; end
                                        end
                                    else if(h_cnt<=320&&v_cnt<leftupbd)
                                        begin
                                            {vgaRed,vgaGreen,vgaBlue}=pixel;
                                            nextstate=`SPLIT;
                                            nextmidbd=midbd-1;
                                            nextrightupbd=rightupbd+1;
                                            nextrightdwbd=rightdwbd+1;
                                            if(leftupbd>0) begin nextleftupbd=leftupbd-1; end
                                            else begin nextleftupbd=leftupbd; end
                                        end
                                    else if(h_cnt<=320&&v_cnt>=leftupbd&&v_cnt<=240)
                                        begin
                                            {vgaRed,vgaGreen,vgaBlue}=12'h0;
                                            nextstate=`SPLIT;
                                            nextmidbd=midbd-1;
                                            nextrightupbd=rightupbd+1;
                                            nextrightdwbd=rightdwbd+1;
                                            if(leftupbd>0) begin nextleftupbd=leftupbd-1; end
                                            else begin nextleftupbd=leftupbd; end    
                                        end
                                    else if(h_cnt>rightupbd&&v_cnt<=240)
                                        begin
                                            {vgaRed,vgaGreen,vgaBlue}=pixel;
                                            nextstate=`SPLIT;
                                            nextmidbd=midbd-1;
                                            nextrightupbd=rightupbd+1;
                                            nextrightdwbd=rightdwbd+1;
                                            if(leftupbd>0) begin nextleftupbd=leftupbd-1; end
                                            else begin nextleftupbd=leftupbd; end   
                                        end
                                    else if(h_cnt<=rightupbd&&v_cnt<=240)
                                        begin
                                            {vgaRed,vgaGreen,vgaBlue}=12'h0;
                                            nextstate=`SPLIT;
                                            nextmidbd=midbd-1;
                                            nextrightupbd=rightupbd+1;
                                            nextrightdwbd=rightdwbd+1;
                                            if(leftupbd>0) begin nextleftupbd=leftupbd-1; end
                                            else begin nextleftupbd=leftupbd; end
                                        end
                                    else if(h_cnt>320&&v_cnt>240&&v_cnt<=rightdwbd)
                                        begin
                                            {vgaRed,vgaGreen,vgaBlue}=12'h0;
                                            nextstate=`SPLIT;
                                            nextmidbd=midbd-1;
                                            nextrightupbd=rightupbd+1;
                                            nextrightdwbd=rightdwbd+1;
                                            if(leftupbd>0) begin nextleftupbd=leftupbd-1; end
                                            else begin nextleftupbd=leftupbd; end
                                        end
                                    else if(h_cnt>321&&v_cnt>rightdwbd)
                                        begin
                                            {vgaRed,vgaGreen,vgaBlue}=pixel;
                                            nextstate=`SPLIT;
                                            nextmidbd=midbd-1;
                                            nextrightupbd=rightupbd+1;
                                            nextrightdwbd=rightdwbd+1;
                                            if(leftupbd>0) begin nextleftupbd=leftupbd-1; end
                                            else begin nextleftupbd=leftupbd; end
                                        end
                                    /*else if(h_cnt>midbd&&v_cnt>240)
                                        begin
                                            {vgaRed,vgaGreen,vgaBlue}=12'h0;
                                            nextstate=`SPLIT;
                                            nextmidbd=midbd-1;
                                            nextrightupbd=rightupbd+1;
                                            nextrightdwbd=rightdwbd+1;
                                           if(leftupbd>0) begin nextleftupbd=leftupbd-1; end
                                            else begin nextleftupbd=leftupbd; end
                                        end*/
                                    else
                                        begin
                                            {vgaRed,vgaGreen,vgaBlue}=12'h0;
                                            nextstate=`INITIAL;
                                            nextmidbd=midbd-1;
                                            nextleftupbd=leftupbd-1;
                                            nextrightupbd=rightupbd+1;
                                            nextrightdwbd=rightdwbd+1;
                                        end
                                end
                        end
                    default:
                        begin
                            nextrightbd=rightbd;
                            nextdwbd=dwbd;
                            nextstate=state;
                            nextmidbd=midbd;
                            nextleftupbd=leftupbd;
                            nextrightupbd=rightupbd;
                            nextrightdwbd=rightdwbd;
                            {vgaRed,vgaGreen,vgaBlue}=12'h0;
                        end
                endcase
            end
    end
    always@(posedge clk20 or posedge rst)
        begin
            if(rst) 
                begin
                    rightbd<=640; 
                    dwbd<=0;
                    midbd<=320;
                    leftupbd<=240;
                    rightupbd<=320;
                    rightdwbd<=240;
                    state<=`INITIAL;
                end
            else 
                begin 
                    rightbd<=nextrightbd;
                    dwbd<=nextdwbd;
                    leftupbd<=nextleftupbd;
                    rightupbd<=nextrightupbd;
                    rightdwbd<=nextrightdwbd;
                    midbd<=nextmidbd;
                    state<=nextstate;
                end
        end
    /*always@(posedge clk21 or posedge rst)
        begin
            if(rst) 
                begin
                    leftupbd<=240;
                end
            else 
                begin 
                    leftupbd<=nextleftupbd;
                end
        end*/
     clock_divisor clk_wiz_0_inst(
      .clk(clk),
      .clk1(clk_25MHz),
      .clk22(clk_22)
    );

    mem_addr_gen mem_addr_gen_inst(
    .clk(clk22),
    .rst(rst),
    .h_cnt(h_cnt),
    .v_cnt(v_cnt),
    .pixel_addr(pixel_addr),
    .state(state),
    .clk21(clk21),
    .clk23(clk23),
    .leftupbd(leftupbd)
    );
     
 
    blk_mem_gen_0 blk_mem_gen_0_inst(
      .clka(clk_25MHz),
      .wea(0),
      .addra(pixel_addr),
      .dina(data[11:0]),
      .douta(pixel)
    ); 

    vga_controller   vga_inst(
      .pclk(clk_25MHz),
      .reset(rst),
      .hsync(hsync),
      .vsync(vsync),
      .valid(valid),
      .h_cnt(h_cnt),
      .v_cnt(v_cnt)
    );
endmodule

module clock_divisor(clk1, clk, clk22);
input clk;
output clk1;
output clk22;
reg [21:0] num;
wire [21:0] next_num;

always @(posedge clk) begin
  num <= next_num;
end

assign next_num = num + 1'b1;
assign clk1 = num[1];
assign clk22 = num[21];
endmodule

module myclk_div(clk,clk_div);
    parameter n=4;
    input clk;
    output clk_div;
    reg [n-1:0]num;
    wire [n-1:0]nextnum;
    always@(posedge clk) begin num<=nextnum; end
    assign nextnum=num+1;
    assign clk_div=num[n-1];

endmodule

module mem_addr_gen(
   input clk,
   input rst,
   input clk21,
   input clk23,
   input [9:0]leftupbd,
   input [9:0] h_cnt,
   input [9:0] v_cnt,
   input [1:0]state,
   output [16:0] pixel_addr
   );
    
   reg [8:0] position;
   reg [8:0] nextposition;
   reg [8:0] position2;
   reg [8:0] nextposition2;
   reg [8:0] position3;
   reg [8:0] nextposition3;
   reg [8:0] position4;
   reg [8:0] nextposition4;
  
   //assign pixel_addr = ((h_cnt>>1)+320*(v_cnt>>1)+ position*320 )% 76800;  //640*480 --> 320*240 
   assign pixel_addr = (h_cnt<320&&v_cnt>240)?((h_cnt>>1)+320*(v_cnt>>1)+position)%76800:
                        (h_cnt<320&&v_cnt<=240)?((h_cnt>>1)+320*(v_cnt>>1)+position2*320)%76800:
                        (h_cnt>=320&&v_cnt<=240)?((h_cnt>>1)+320*(v_cnt>>1)+position3)%76800:
                        (h_cnt>=320&&v_cnt>240)?((h_cnt>>1)+320*(v_cnt>>1)+position4*320)%76800:
                        ((h_cnt>>1)+320*(v_cnt>>1))%76800;  //640*480 --> 320*240 
    always@(*)
        begin
            if(state==`SPLIT)
                begin
                    if(position<159) begin nextposition=position+1; end
                    else begin nextposition=0; end
                    if(position2<119) begin nextposition2=position2+1; end
                    else begin nextposition2=0; end
                    if(position3>159) begin nextposition3=position3-1; end
                    else begin nextposition3=319; end
                    if(position4>119) begin nextposition4=position4-1; end
                    else begin nextposition4=239; end
                end
            else
                begin
                    nextposition=0;
                    nextposition2=0;
                    nextposition3=319;
                    nextposition4=239;
                end
        end
    always@(posedge clk21,posedge rst)
        begin
            if(rst)
                begin
                    position<=0;
                    position2<=0;
                    position3<=319;
                    position4<=239;
                end
            else
                begin
                    position<=nextposition;
                    position2<=nextposition2;
                    position3<=nextposition3;
                    position4<=nextposition4;
                end
        end
    
endmodule

`timescale 1ns/1ps
/////////////////////////////////////////////////////////////////
// Module Name: vga
/////////////////////////////////////////////////////////////////

module vga_controller (
    input wire pclk, reset,
    output wire hsync, vsync, valid,
    output wire [9:0]h_cnt,
    output wire [9:0]v_cnt
    );

    reg [9:0]pixel_cnt;
    reg [9:0]line_cnt;
    reg hsync_i,vsync_i;

    parameter HD = 640;
    parameter HF = 16;
    parameter HS = 96;
    parameter HB = 48;
    parameter HT = 800; 
    parameter VD = 480;
    parameter VF = 10;
    parameter VS = 2;
    parameter VB = 33;
    parameter VT = 525;
    parameter hsync_default = 1'b1;
    parameter vsync_default = 1'b1;

    always @(posedge pclk)
        if (reset)
            pixel_cnt <= 0;
        else
            if (pixel_cnt < (HT - 1))
                pixel_cnt <= pixel_cnt + 1;
            else
                pixel_cnt <= 0;

    always @(posedge pclk)
        if (reset)
            hsync_i <= hsync_default;
        else
            if ((pixel_cnt >= (HD + HF - 1)) && (pixel_cnt < (HD + HF + HS - 1)))
                hsync_i <= ~hsync_default;
            else
                hsync_i <= hsync_default; 

    always @(posedge pclk)
        if (reset)
            line_cnt <= 0;
        else
            if (pixel_cnt == (HT -1))
                if (line_cnt < (VT - 1))
                    line_cnt <= line_cnt + 1;
                else
                    line_cnt <= 0;

    always @(posedge pclk)
        if (reset)
            vsync_i <= vsync_default; 
        else if ((line_cnt >= (VD + VF - 1)) && (line_cnt < (VD + VF + VS - 1)))
            vsync_i <= ~vsync_default; 
        else
            vsync_i <= vsync_default; 

    assign hsync = hsync_i;
    assign vsync = vsync_i;
    assign valid = ((pixel_cnt < HD) && (line_cnt < VD));

    assign h_cnt = (pixel_cnt < HD) ? pixel_cnt : 10'd0;
    assign v_cnt = (line_cnt < VD) ? line_cnt : 10'd0;

endmodule






