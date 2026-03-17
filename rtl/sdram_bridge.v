// AXI to Avalon Bridge
// Converts AXI4 bursts to 16-bit Avalon accesses
// AXI Slave interface (64-bit) to Avalon Master interface (16-bit)
// FINAL VERSION - with write timing matching read timing

module axi_burst_master_to_avalon16 #(
    parameter AXI_ADDR_WIDTH = 32,
    parameter AXI_DATA_WIDTH = 64,
    parameter AVALON_ADDR_WIDTH = 25,
    parameter AVALON_DATA_WIDTH = 16,
    parameter AXI_MAX_BURST = 16,
    parameter FIFO_DEPTH = 32
) (
    // Clock and reset
    input  wire                        clk,
    input  wire                        reset_n,
    
    // AXI4 Slave Interface
    input  wire [AXI_ADDR_WIDTH-1:0]   s_awaddr,
    input  wire [7:0]                   s_awlen,
    input  wire [2:0]                   s_awsize,
    input  wire [1:0]                   s_awburst,
    input  wire                         s_awvalid,
    output reg                          s_awready,
    
    // Write Data Channel
    input  wire [AXI_DATA_WIDTH-1:0]   s_wdata,
    input  wire [AXI_DATA_WIDTH/8-1:0] s_wstrb,
    input  wire                         s_wlast,
    input  wire                         s_wvalid,
    output reg                          s_wready,
    
    // Write Response Channel
    output reg  [1:0]                   s_bresp,
    output reg                          s_bvalid,
    input  wire                         s_bready,
    
    // Read Address Channel
    input  wire [AXI_ADDR_WIDTH-1:0]   s_araddr,
    input  wire [7:0]                   s_arlen,
    input  wire [2:0]                   s_arsize,
    input  wire [1:0]                   s_arburst,
    input  wire                         s_arvalid,
    output reg                          s_arready,
    
    // Read Data Channel
    output reg  [AXI_DATA_WIDTH-1:0]   s_rdata,
    output reg  [1:0]                   s_rresp,
    output reg                          s_rlast,
    output reg                          s_rvalid,
    input  wire                         s_rready,
    
    // Avalon Master Interface
    output reg  [AVALON_ADDR_WIDTH-1:0] m_address,
    output reg                          m_write,
    output reg                          m_read,
    output reg  [AVALON_DATA_WIDTH-1:0] m_writedata,
    input  wire [AVALON_DATA_WIDTH-1:0] m_readdata,
    output reg  [1:0]                    m_byteenable,
    input  wire                          m_waitrequest,
    input  wire                          m_readdatavalid,
    output reg                           m_burstcount
);

// ====================================================
// Local Parameters
// ====================================================
localparam BEATS_PER_AXI = AXI_DATA_WIDTH / AVALON_DATA_WIDTH;  // 64/16 = 4
localparam STRB_PER_AXI = AXI_DATA_WIDTH / 8;                   // 64/8 = 8
localparam STRB_PER_AVALON = AVALON_DATA_WIDTH / 8;             // 16/8 = 2

// ====================================================
// States
// ====================================================
localparam IDLE         = 4'd0;
localparam WRITE_DATA   = 4'd2;
localparam WRITE_ADDR   = 4'd3;      // New: set up write address/data (like READ_ADDR)
localparam WRITE_AVALON = 4'd4;      // Assert write (like READ_AVALON)
localparam WRITE_RESP   = 4'd5;
localparam READ_ADDR    = 4'd6;
localparam READ_AVALON  = 4'd7;
localparam READ_COMBINE = 4'd8;
localparam READ_DATA    = 4'd9;

// ====================================================
// Internal Registers and Wires
// ====================================================
reg [3:0] state;

// Write FIFO
reg [AVALON_ADDR_WIDTH-1:0] write_addr_fifo [0:FIFO_DEPTH-1];
reg [AVALON_DATA_WIDTH-1:0] write_data_fifo [0:FIFO_DEPTH-1];
reg [1:0]                   write_strb_fifo [0:FIFO_DEPTH-1];
reg [7:0]                   write_head;
reg [7:0]                   write_tail;
wire                        write_fifo_empty;
wire                        write_fifo_full;

// Write tracking
reg [AXI_ADDR_WIDTH-1:0]    write_base_addr;
reg [7:0]                   write_len;
reg [7:0]                   write_beat_count;
reg [7:0]                   write_total_beats;
reg                         write_burst_active;
reg                         write_response_pending;
reg                         write_resp_sent;

// Write pipeline registers (like read_addr_reg/read_valid_reg)
reg [AVALON_ADDR_WIDTH-1:0] write_addr_reg;
reg [AVALON_DATA_WIDTH-1:0] write_data_reg;
reg [1:0]                   write_be_reg;
reg                         write_valid_reg;

// Read tracking
reg [AXI_ADDR_WIDTH-1:0]    read_base_addr;
reg [7:0]                   read_len;
reg [7:0]                   read_beat_count;
reg [2:0]                   read_sub_count;
reg [15:0]                  read_hold [0:3];
reg                         read_burst_active;

// Read address pipeline
reg [AVALON_ADDR_WIDTH-1:0] read_addr_reg;
reg                         read_valid_reg;
reg [2:0]                   read_sub_reg;

// Loop variables
integer i;
integer fifo_idx;

// Timeout counters
reg [15:0] write_timeout;
reg [15:0] read_timeout;

// ====================================================
// FIFO Status
// ====================================================
assign write_fifo_empty = (write_head == write_tail);
assign write_fifo_full = (((write_tail + 1) % FIFO_DEPTH) == write_head);

// ====================================================
// Main State Machine
// ====================================================
always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        state <= IDLE;
        s_awready <= 1'b0;
        s_wready <= 1'b0;
        s_bvalid <= 1'b0;
        s_bresp <= 2'b00;
        s_arready <= 1'b0;
        s_rvalid <= 1'b0;
        s_rlast <= 1'b0;
        s_rdata <= 64'b0;
        m_write <= 1'b0;
        m_read <= 1'b0;
        m_byteenable <= 2'b00;
        m_address <= 0;
        m_writedata <= 0;
        write_head <= 0;
        write_tail <= 0;
        write_beat_count <= 0;
        write_total_beats <= 0;
        write_burst_active <= 1'b0;
        write_response_pending <= 1'b0;
        write_resp_sent <= 1'b0;
        write_valid_reg <= 1'b0;
        write_addr_reg <= 0;
        write_data_reg <= 0;
        write_be_reg <= 0;
        read_beat_count <= 0;
        read_sub_count <= 0;
        read_burst_active <= 1'b0;
        read_addr_reg <= 0;
        read_valid_reg <= 1'b0;
        for (i = 0; i < 4; i = i + 1) read_hold[i] <= 16'b0;
        write_timeout <= 0;
        read_timeout <= 0;
    end else begin
        case (state)
            // ========================================
            IDLE: begin
                s_awready <= 1'b1;
                s_arready <= 1'b1;
                s_wready <= 1'b0;
                m_write <= 1'b0;
                m_read <= 1'b0;
                write_timeout <= 0;
                read_timeout <= 0;
                write_valid_reg <= 1'b0;
                
                // If there's a pending write response not yet sent, enter WRITE_RESP
                if (write_response_pending && !write_resp_sent) begin
                    state <= WRITE_RESP;
                end
                // Check for write request
                else if (s_awvalid && s_awready) begin
                    // Write request
                    write_base_addr <= s_awaddr;
                    write_len <= s_awlen;
                    write_total_beats <= s_awlen + 1;
                    write_beat_count <= 0;
                    write_burst_active <= 1'b1;
                    write_resp_sent <= 1'b0;
                    s_awready <= 1'b0;
                    s_arready <= 1'b0;
                    $display("  [BRIDGE] Write request: addr=%h len=%0d", s_awaddr, s_awlen);
                    state <= WRITE_DATA;
                end
                // Check for read request
                else if (s_arvalid && s_arready) begin
                    // Read request
                    read_base_addr <= s_araddr;
                    read_len <= s_arlen;
                    read_beat_count <= 0;
                    read_sub_count <= 0;
                    read_burst_active <= 1'b1;
                    s_arready <= 1'b0;
                    s_awready <= 1'b0;
                    $display("  [BRIDGE] Read request: addr=%h len=%0d", s_araddr, s_arlen);
                    state <= READ_ADDR;
                end
            end
            
            // ========================================
            // WRITE: Receive AXI data
            // ========================================
            WRITE_DATA: begin
                s_wready <= 1'b1;
                
                if (s_wvalid && s_wready) begin
                    $display("  [BRIDGE] Write data received: beat=%0d data=%h", 
                             write_beat_count, s_wdata);
                    
                    // Decompose 64-bit data into 4 16-bit words
                    for (i = 0; i < BEATS_PER_AXI; i = i + 1) begin
                        fifo_idx = (write_tail + i) % FIFO_DEPTH;
                        
                        // Store 16-bit word
                        write_data_fifo[fifo_idx] <= s_wdata[i * 16 +: 16];
                        
                        // Store byte enable
                        write_strb_fifo[fifo_idx] <= s_wstrb[i * 2 +: 2];
                        
                        // Calculate address: convert byte address to word address
                        write_addr_fifo[fifo_idx] <= (write_base_addr >> 1) + 
                                                     (write_beat_count * BEATS_PER_AXI) + i;
                        
                        $display("  [BRIDGE]   FIFO[%0d]: addr=%h data=%h be=%b", 
                                 fifo_idx,
                                 (write_base_addr >> 1) + (write_beat_count * BEATS_PER_AXI) + i,
                                 s_wdata[i * 16 +: 16],
                                 s_wstrb[i * 2 +: 2]);
                    end
                    
                    write_tail <= write_tail + BEATS_PER_AXI;
                    write_beat_count <= write_beat_count + 1;
                    
                    if (s_wlast) begin
                        $display("  [BRIDGE] Last write data received");
                        s_wready <= 1'b0;
                        state <= WRITE_ADDR;  // Go to WRITE_ADDR first
                    end
                end
            end
            
            // ========================================
            // WRITE: Set up address and data (like READ_ADDR)
            // ========================================
            WRITE_ADDR: begin
                if (!write_fifo_empty) begin
                    // Set up address and data registers
                    write_addr_reg <= write_addr_fifo[write_head];
                    write_data_reg <= write_data_fifo[write_head];
                    write_be_reg <= write_strb_fifo[write_head];
                    write_valid_reg <= 1'b1;
                    
                    $display("  [BRIDGE] WRITE_ADDR: addr=%h data=%h be=%b", 
                             write_addr_fifo[write_head], 
                             write_data_fifo[write_head], 
                             write_strb_fifo[write_head]);
                    
                    state <= WRITE_AVALON;
                end
            end
            
            // ========================================
            // WRITE: Assert write (like READ_AVALON)
            // ========================================
            WRITE_AVALON: begin
                // Drive address and data from registers and assert write
                if (write_valid_reg) begin
                    m_address <= write_addr_reg;
                    m_writedata <= write_data_reg;
                    m_byteenable <= write_be_reg;
                    m_write <= 1'b1;
                    write_valid_reg <= 1'b0;
                    $display("  [BRIDGE] WRITE_AVALON: asserting write at addr=%h data=%h be=%b", 
                             write_addr_reg, write_data_reg, write_be_reg);
                end
                
                // Wait for acceptance
                if (!m_waitrequest && m_write) begin
                    $display("  [BRIDGE] Write accepted at time %t", $time);
                    m_write <= 1'b0;
                    write_head <= write_head + 1;
                    write_timeout <= 0;
                    
                    // Check if all writes are done
                    if (write_head == (write_tail - 1)) begin
                        $display("  [BRIDGE] All writes completed at time %t", $time);
                        write_response_pending <= 1'b1;
                        state <= IDLE;
                    end else begin
                        // More writes to do
                        state <= WRITE_ADDR;
                    end
                end else begin
                    if (write_timeout == 16'hFFFF) begin
                        $display("  [BRIDGE] Write timeout!");
                        write_response_pending <= 1'b1;
                        state <= IDLE;
                        write_timeout <= 0;
                    end else begin
                        write_timeout <= write_timeout + 1;
                    end
                end
            end
            
            // ========================================
            // WRITE: Send response
            // ========================================
            WRITE_RESP: begin
                if (!s_bvalid) begin
                    s_bresp <= 2'b00;  // OKAY
                    s_bvalid <= 1'b1;
                    write_resp_sent <= 1'b1;
                    $display("  [BRIDGE] Sending write response at time %t", $time);
                end
                
                if (s_bvalid && s_bready) begin
                    $display("  [BRIDGE] Write response accepted at time %t", $time);
                    s_bvalid <= 1'b0;
                    s_awready <= 1'b1;
                    s_arready <= 1'b1;
                    write_burst_active <= 1'b0;
                    write_response_pending <= 1'b0;
                    write_resp_sent <= 1'b0;
                    state <= IDLE;
                end
                
                if (write_timeout == 16'hFFFF) begin
                    $display("  [BRIDGE] Write response timeout!");
                    s_bvalid <= 1'b0;
                    write_response_pending <= 1'b0;
                    write_resp_sent <= 1'b0;
                    state <= IDLE;
                    write_timeout <= 0;
                end else begin
                    write_timeout <= write_timeout + 1;
                end
            end
            
            // ========================================
            // READ: Set up address
            // ========================================
            READ_ADDR: begin
                read_addr_reg <= (read_base_addr >> 1) + 
                                 (read_beat_count * BEATS_PER_AXI) + read_sub_count;
                read_sub_reg <= read_sub_count;
                read_valid_reg <= 1'b1;
                
                $display("  [BRIDGE] READ_ADDR: beat=%0d sub=%0d addr=0x%h", 
                         read_beat_count, read_sub_count, read_addr_reg);
                
                state <= READ_AVALON;
            end
            
            // ========================================
            // READ: Assert read and collect data
            // ========================================
            READ_AVALON: begin
                if (read_valid_reg) begin
                    m_address <= read_addr_reg;
                    m_read <= 1'b1;
                    read_valid_reg <= 1'b0;
                    $display("  [BRIDGE] READ_AVALON: asserting read at addr=0x%h", read_addr_reg);
                end
                
                if (!m_waitrequest && m_read) begin
                    m_read <= 1'b0;
                end
                
                if (m_readdatavalid) begin
                    $display("  [BRIDGE] Read data received: sub=%0d data=%h", read_sub_reg, m_readdata);
                    
                    // Store data in order of arrival
                    read_hold[read_sub_reg] <= m_readdata;
                    
                    if (read_sub_reg == BEATS_PER_AXI - 1) begin
                        state <= READ_COMBINE;
                    end else begin
                        read_sub_count <= read_sub_count + 1;
                        state <= READ_ADDR;
                    end
                    read_timeout <= 0;
                end else begin
                    if (read_timeout == 16'hFFFF) begin
                        $display("  [BRIDGE] Read timeout!");
                        state <= IDLE;
                        read_timeout <= 0;
                    end else begin
                        read_timeout <= read_timeout + 1;
                    end
                end
            end
            
            // ========================================
            // READ: Combine data
            // ========================================
            READ_COMBINE: begin
                // Little-endian: hold[0] is LSB, hold[3] is MSB
                s_rdata <= {read_hold[3], read_hold[2], read_hold[1], read_hold[0]};
                s_rvalid <= 1'b1;
                
                $display("  [BRIDGE] Combining: {%h, %h, %h, %h} = %h", 
                         read_hold[3], read_hold[2], read_hold[1], read_hold[0],
                         {read_hold[3], read_hold[2], read_hold[1], read_hold[0]});
                
                if (read_beat_count == read_len) begin
                    s_rlast <= 1'b1;
                    $display("  [BRIDGE] Last beat");
                end
                
                read_beat_count <= read_beat_count + 1;
                read_sub_count <= 0;
                state <= READ_DATA;
                read_timeout <= 0;
            end
            
            // ========================================
            // READ: Send to AXI
            // ========================================
            READ_DATA: begin
                if (s_rready) begin
                    s_rvalid <= 1'b0;
                    s_rlast <= 1'b0;
                    
                    if (read_beat_count > read_len) begin
                        // Burst complete
                        read_burst_active <= 1'b0;
                        s_arready <= 1'b1;
                        s_awready <= 1'b1;
                        state <= IDLE;
                    end else begin
                        // Continue with next beat
                        read_sub_count <= 0;
                        state <= READ_ADDR;
                    end
                end
            end
            
            default: begin
                state <= IDLE;
            end
        endcase
    end
end

// ====================================================
// Output assignments
// ====================================================
always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        m_burstcount <= 1'b0;
        s_rresp <= 2'b00;
    end else begin
        m_burstcount <= 1'b1;
        s_rresp <= 2'b00;
    end
end

// ====================================================
// Debug monitor (synthesis ignores)
// ====================================================
// synthesis translate_off
always @(posedge clk) begin
    if (m_write) begin
        $display("  [BRIDGE] Avalon Write: addr=0x%h data=0x%h be=%b at time %t", 
                 m_address, m_writedata, m_byteenable, $time);
    end
    if (m_read) begin
        $display("  [BRIDGE] Avalon Read: addr=0x%h at time %t", 
                 m_address, $time);
    end
end
// synthesis translate_on

endmodule
