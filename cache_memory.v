`timescale 1ns / 1ps

module cache_controller(
input               clk,
input               reset,
input               cache_rd_wr,// read/write signal 
input               cpu_valid,
input       [31:0]  cpu_add,
input       [7:0]   cache_cpu_in,
input       [7:0]   cache_mem_in,

// output reg mem_ready,
output reg          mem_rd_wr,
output reg  [31:0]  mem_add,
output reg  [7:0]   cache_cpu_out,
output reg  [7:0]   cache_mem_out,
output reg          mem_valid,
output reg          cache_ready,
output reg  [31:0]  total_hits,
output reg  [31:0]  total_misses
);
// parametrizing cache memory variables
parameter cache_size    = 65536; // 64KB by default
parameter block_size    = 16;    // 16B by defualt 
parameter associativity = 1;     // default associativity = 1 (direct mapped)
parameter cache_lines   = (cache_size) / (block_size * associativity);

// calculating number of tag, index, offset bits from cache variables
parameter offset_bits = $clog2(block_size);//finding the number of bits for block offset
parameter index_bits  = $clog2(cache_lines);//finding the number of bits for index
parameter tag_bits    = 32 - offset_bits - index_bits;//finding the number of bits for tag

// designing cache memory with tag, valid, dirty bit arrays
reg [(block_size)*8-1:0] cache_data [0:cache_lines-1];
reg [tag_bits-1:0]       tag_array  [0:cache_lines-1];
reg                      valid      [0:cache_lines-1];
reg                      dirty      [0:cache_lines-1];

// initializing intermediate signals

reg                      hit;
reg                      miss;
reg [offset_bits-1:0]    mem_data_counter;// counter to keep count of the byte transfered in block transfer
reg [1:0] present_state, next_state;
integer k;

// decoding input address into tag, index, byte_offset
wire [tag_bits-1:0]      tag;
wire [index_bits-1:0]    index;
wire [offset_bits-1:0]   byte_offset;
assign {tag, index, byte_offset} = cpu_add;

// FSM states for cache
parameter idle       = 2'd0;
parameter compare    = 2'd1;
parameter write_back = 2'd2;
parameter allocate   = 2'd3;

//sequential block 
always @(posedge clk or posedge reset) begin
if (reset) begin
// reset logic to clear the entire cache along with signals
present_state      <= idle;
total_hits         <= 0;
total_misses         <= 0;
mem_data_counter   <= 0;

cache_mem_out  <= 0;
cache_ready    <= 1'b0;       
// loop to clear dirty and valid bits of cache memory
for (k = 0; k < cache_lines; k = k + 1) begin
    dirty[k]      <= 1'b0;
    valid[k]      <= 1'b0;
    tag_array[k]  <= 0;
end
end else begin
present_state <= next_state;
case (present_state)
    idle:begin
    cache_cpu_out<=0;
        cache_ready  <= 1'b0;
        cache_mem_out<=0;
        end
    compare: begin
        if (hit) begin
            total_hits <= total_hits + 1;                               // for a hit update hit count
            if (cache_rd_wr) begin                                       // read data from cache
                cache_cpu_out <= cache_data[index][(byte_offset)*8 +: 8];// transfer data from cache to cpu
                cache_ready   <= 1'b1;                                       //indicate that data is trasnfered succesfully 
            end else begin                                                  // write data to cache
                cache_data[index][(byte_offset)*8 +: 8] <= cache_cpu_in;        
                dirty[index] <= 1'b1;                                          // set dirty bit to indicate cache write_back
                cache_ready  <= 1'b1;                                                  
            end
        end else if (miss) begin
        cache_cpu_out<=0;
            total_misses <= total_misses + 1;                           //if miss update miss count
        end
    end
    write_back: begin // if a dirty miss tranfer data to memory first 
        // whole block needs to be replaced; byte_offset is variable
        cache_ready  <= 1'b0;
        cache_cpu_out<=0;
        if (mem_data_counter < block_size) begin
            cache_mem_out <= cache_data[index][mem_data_counter*8 +: 8];// transfer one byte per cyle 
            if (mem_data_counter == block_size-1) begin
                dirty[index]      <= 1'b0; // clean the cache line by whole block is transfered
                mem_data_counter  <= 0;
            end else begin
                mem_data_counter <= mem_data_counter + 1;
            end
        end
    end
    allocate: begin    // if a clean miss transfer required data from memory to cache
        cache_ready  <= 1'b0;
        cache_cpu_out<=0;
        cache_mem_out<=0;
        if (mem_data_counter < block_size) begin
            cache_data[index][mem_data_counter*8 +: 8] <= cache_mem_in;
            if (mem_data_counter == block_size-1) begin
                mem_data_counter   <= 0;
                tag_array[index]   <= tag;  // replace current tag with the intended tag
                valid[index]       <= 1'b1; // set valid bit
                dirty[index]       <= 1'b0; // clean the cache line
            end else begin
                mem_data_counter <= mem_data_counter + 1;
            end
        end
    end
    
endcase
end
end

// Combinational block 
always @(*) begin
// assigning default signals 
next_state     = present_state;
hit            = 1'b0;
miss           = 1'b0;
mem_rd_wr      = 1'b0;       // default memory write
mem_add        = 0;
mem_valid      = 1'b0;

case (present_state)
idle: begin
    if (cpu_valid) // move to compare state if CPU request arrives
        next_state = compare;
end

compare: begin
    if (tag_array[index] == tag && valid[index] == 1) begin // check for hit or miss
        hit        = 1'b1; // set hit bit to indicate hit
        next_state = idle; // return to idle after hit
    end else begin
        // miss
        miss = 1'b1; // it is a miss
        if (dirty[index]) begin
            // dirty: must write back first
            next_state = write_back;
        end else begin
            // not dirty: allocate directly
            next_state = allocate;
        end
    end
end

write_back: begin
    mem_rd_wr = 1'b0;        // active-low write signal
    mem_valid = 1'b1;        // active-high valid signal
    mem_add   = {tag_array[index], index, mem_data_counter}; // update memory address depending on current byte which is being transfered 
    if (mem_data_counter == block_size-1) begin  //  if block transfer completed then off the memory connection 
        mem_valid  = 1'b0;
        next_state = allocate; // after writing to main memory, go to allocate
    end
end

allocate: begin
    mem_rd_wr = 1'b1;        // active-high read signal
    mem_valid = 1'b1;        // active-high valid signal
    mem_add   = {tag, index, mem_data_counter}; // current cache line address
    if (mem_data_counter == block_size-1) begin
        mem_valid  = 1'b0;
        next_state = compare; // after reading from main memory, go to compare
    end
end

default:
    next_state = idle;
endcase
end
endmodule
