`timescale 1ns / 1ps

module testbench();
reg clk;
reg reset;
reg cache_rd_wr;
reg cpu_valid;
reg[31:0] cpu_add;
reg[7:0]   cache_cpu_in;
reg[7:0]   cache_mem_in;


    wire          mem_rd_wr;
    wire  [31:0]  mem_add;
    wire  [7:0]   cache_cpu_out;
    wire  [7:0]   cache_mem_out;
    wire          mem_valid;
    wire          cache_ready;
    wire  [31:0]  total_hits;
    wire  [31:0]  total_misses;
   
integer k;
cache_controller dut(.clk(clk),.reset(reset),.cache_rd_wr(cache_rd_wr),.cpu_valid(cpu_valid),.cpu_add(cpu_add),.cache_cpu_in(cache_cpu_in),.cache_mem_in(cache_mem_in),
.mem_rd_wr(mem_rd_wr),.mem_add(mem_add),.cache_cpu_out(cache_cpu_out),.cache_mem_out(cache_mem_out),.mem_valid(mem_valid),.cache_ready(cache_ready),.total_hits(total_hits),.total_misses(total_misses)
);

defparam dut.cache_size=65536;
defparam dut.block_size=16;
defparam dut.associativity=1;

initial 
begin 
clk=0;
forever #5 clk=~clk;
end

initial 
begin 
cache_rd_wr=0;
cpu_valid=0;
cpu_add=32'b0;
cache_cpu_in=8'b0;
cache_mem_in=8'b0;
end

initial 
begin 
reset=1;
#12 reset=0;
end

initial 
begin 
//@(negedge reset)
// first test for read miss
@(negedge reset)
//@(posedge clk)// at fist clock edge give cpu request and move to compare
#1
cpu_add=32'h111aaaab;// first three digits for tag next four for index last for offset for default case 
cpu_valid=1;
cache_rd_wr=1;// read request 

 @(negedge clk) // from clean miss move to allocate at this clk edge

cpu_valid=0;//
@(posedge clk)
for(k=0;k<16;k=k+1)
begin
#1
cache_mem_in=k;
@(posedge clk);
end 

 // after block transfer from memory to cache state becomes compare 
@(posedge clk) // at this clock edge state becomes idle and data is read 
cache_mem_in=0;
// now again reading data from same address for hit 
@(posedge clk) //one clock cycle delay and move to compare state
@(negedge clk)
cpu_add=32'h111aaaab;
cpu_valid=1;
cache_rd_wr=1;// read request

@(negedge clk)// moves to again idle state after hit 
cpu_valid=0;
@(posedge clk)
@(posedge clk)
@(negedge clk)
// now testing for write-hit updating the same cache line
cpu_add=32'h111aaaab; 
cpu_valid=1;
cache_rd_wr=0;// write request
cache_cpu_in=8'hAA;
@(negedge clk) // data is written and dirty bit is high for tag 111 
cpu_valid=0;
@(posedge clk)
// now reading from same cache line to check if written successfully 
@(posedge clk)
@(negedge clk)
cache_cpu_in=8'h00;
cpu_add=32'h111aaaab; 
cpu_valid=1;
cache_rd_wr=1;// read request
@(negedge clk) 
cpu_valid=0;
@(posedge clk )

// now to read data from same cache line different tag (read miss + dirty)
@(negedge clk)
cpu_add=32'h222aaaab; 
cpu_valid=1;
cache_rd_wr=1;// read request
@(negedge clk)// dirty miss moves to write_back state  
cpu_valid=0;
@(negedge mem_valid) // wait till whole block is transfered with byte transfer per cycle

@(posedge clk) // when counter=15 after one cycle it will move to allocate state
for(k=0;k<16;k=k+1)
begin 
#1
cache_mem_in=8'd100+k;
@(posedge clk);
end
 

////$display("total hits=%d  total misses=%d",total_hits,total_misses);
#20 $finish;
end

initial 
begin 
$monitor("time=%t clock=%b   reset=%b   cpu_valid=%b    mem_address=%h    mem_rd/wr=%b   cache_output_data_to_cpu=%h  cache_output_data_to_mem=%h mem_valid=%b  cache_ready=%b   hits=%d   misses=%d ",$time ,clk,reset,cpu_valid,mem_add,mem_rd_wr,cache_cpu_out,cache_mem_out, mem_valid,cache_ready,total_hits,total_misses);


end
endmodule
