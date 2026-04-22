// AXI to Avalon Bridge with ID support
// Converts AXI4 bursts to 16-bit Avalon accesses
// AXI Slave interface (64-bit) to Avalon Master interface (16-bit)

module axi_burst_master_to_avalon16 #(
    parameter AXI_ADDR_WIDTH = 32,
    parameter AXI_DATA_WIDTH = 64,
    parameter AVALON_ADDR_WIDTH = 24,
    parameter AVALON_DATA_WIDTH = 16,
    parameter AXI_MAX_BURST = 16,
    parameter FIFO_DEPTH = 64,          // Increased to hold full burst
    parameter AXI_ID_WIDTH = 4
) (
    input  wire                        clk,
    input  wire                        reset_n,
    
    // AXI4 Slave Interface - Write Address Channel
    input  wire [AXI_ID_WIDTH-1:0]     s_awid,
    input  wire [AXI_ADDR_WIDTH-1:0]   s_awaddr,
    input  wire [7:0]                  s_awlen,
    input  wire [2:0]                  s_awsize,
    input  wire [1:0]                  s_awburst,
    input  wire                        s_awvalid,
    output reg                         s_awready,
    
    // Write Data Channel
    input  wire [AXI_DATA_WIDTH-1:0]   s_wdata,
    input  wire [AXI_DATA_WIDTH/8-1:0] s_wstrb,
    input  wire                        s_wlast,
    input  wire                        s_wvalid,
    output reg                         s_wready,
    
    // Write Response Channel
    output reg  [AXI_ID_WIDTH-1:0]     s_bid,
    output reg  [1:0]                  s_bresp,
    output reg                         s_bvalid,
    input  wire                        s_bready,
    
    // Read Address Channel
    input  wire [AXI_ID_WIDTH-1:0]     s_arid,
    input  wire [AXI_ADDR_WIDTH-1:0]   s_araddr,
    input  wire [7:0]                  s_arlen,
    input  wire [2:0]                  s_arsize,
    input  wire [1:0]                  s_arburst,
    input  wire                        s_arvalid,
    output reg                         s_arready,
    
    // Read Data Channel
    output reg  [AXI_ID_WIDTH-1:0]     s_rid,
    output reg  [AXI_DATA_WIDTH-1:0]   s_rdata,
    output reg  [1:0]                  s_rresp,
    output reg                         s_rlast,
    output reg                         s_rvalid,
    input  wire                        s_rready,
    
    // Avalon Master Interface
    output reg  [AVALON_ADDR_WIDTH-1:0] m_address,
    output reg                          m_write,
    output reg                          m_read,
    output reg  [AVALON_DATA_WIDTH-1:0] m_writedata,
    input  wire [AVALON_DATA_WIDTH-1:0] m_readdata,
    output reg  [1:0]                   m_byteenable,
    input  wire                         m_waitrequest,
    input  wire                         m_readdatavalid,
    output reg                          m_burstcount
);

// ====================================================
// Local Parameters
// ====================================================
localparam BYTES_PER_AVALON = AVALON_DATA_WIDTH / 8;      // 2 bytes
localparam BYTES_PER_AXI    = AXI_DATA_WIDTH / 8;         // 8 bytes
localparam AVALON_PER_AXI   = BYTES_PER_AXI / BYTES_PER_AVALON;  // 4
localparam STRB_PER_AVALON  = AVALON_DATA_WIDTH / 8;      // 2 bits
localparam FIFO_PTR_WIDTH   = $clog2(FIFO_DEPTH);

// ====================================================
// States
// ====================================================
localparam IDLE          = 4'd0;
localparam WRITE_RECV    = 4'd1;   // Receive all AXI beats into FIFO
localparam WRITE_FLUSH   = 4'd2;   // Flush FIFO to Avalon
localparam WRITE_RESP    = 4'd3;   // Send write response
localparam READ_ADDR     = 4'd4;
localparam READ_AVALON   = 4'd5;
localparam READ_COMBINE  = 4'd6;
localparam READ_DATA     = 4'd7;

// ====================================================
// Internal Registers
// ====================================================
reg [3:0] state;

// Write FIFO - stores decomposed 16-bit writes
reg [AXI_ID_WIDTH-1:0]       write_id_fifo [0:FIFO_DEPTH-1];
reg [AVALON_ADDR_WIDTH-1:0]  write_addr_fifo [0:FIFO_DEPTH-1];
reg [AVALON_DATA_WIDTH-1:0]  write_data_fifo [0:FIFO_DEPTH-1];
reg [1:0]                    write_strb_fifo [0:FIFO_DEPTH-1];
reg [FIFO_PTR_WIDTH-1:0]     write_head;
reg [FIFO_PTR_WIDTH-1:0]     write_tail;
wire                         write_fifo_empty;
wire                         write_fifo_full;

// AXI burst tracking
reg [AXI_ID_WIDTH-1:0]       write_axi_id;
reg [AXI_ADDR_WIDTH-1:0]     write_base_addr;            // Saved base address from AW
reg [AXI_ADDR_WIDTH-1:0]     write_base_addr_aligned;    // 8-byte aligned base address
reg [7:0]                    write_axi_total_beats;      // Total AXI beats in burst (awlen+1)
reg [7:0]                    write_axi_beats_received;   // AXI beats received so far
reg                          write_axi_last_beat_received;

// Avalon sub-write tracking
reg [FIFO_PTR_WIDTH-1:0]     write_avalon_total_subwrites;   // Total Avalon sub-writes (total_beats * 4)
reg [FIFO_PTR_WIDTH-1:0]     write_avalon_subwrites_remaining; // Avalon sub-writes remaining to flush

// Response tracking
reg                          write_response_pending;
reg [AXI_ID_WIDTH-1:0]       write_resp_id;

// Avalon write pipeline
reg [AVALON_ADDR_WIDTH-1:0]  write_addr_reg;
reg [AVALON_DATA_WIDTH-1:0]  write_data_reg;
reg [1:0]                    write_be_reg;
reg                          write_valid_reg;

// Read tracking
reg [AXI_ADDR_WIDTH-1:0]     read_base_addr;
reg [AXI_ADDR_WIDTH-1:0]     read_base_addr_aligned;
reg [AXI_ID_WIDTH-1:0]       read_id;
reg [7:0]                    read_len;
reg [7:0]                    read_beat_count;
reg [2:0]                    read_sub_count;
reg [15:0]                   read_hold [0:3];
reg [AVALON_ADDR_WIDTH-1:0]  read_addr_reg;
reg                          read_valid_reg;
reg [2:0]                    read_sub_reg;

// Temporary variables
integer i;
integer fifo_idx;
reg [AXI_ADDR_WIDTH-1:0] word_byte_addr;
reg [AXI_ADDR_WIDTH-1:0] beat_offset;
reg [AXI_ADDR_WIDTH-1:0] word_offset;

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
        // Reset all registers
        state <= IDLE;
        s_awready <= 1'b0;
        s_wready <= 1'b0;
        s_bvalid <= 1'b0;
        s_bresp <= 2'b00;
        s_bid <= {AXI_ID_WIDTH{1'b0}};
        s_arready <= 1'b0;
        s_rvalid <= 1'b0;
        s_rlast <= 1'b0;
        s_rid <= {AXI_ID_WIDTH{1'b0}};
        s_rdata <= 64'b0;
        m_write <= 1'b0;
        m_read <= 1'b0;
        m_byteenable <= 2'b00;
        m_address <= {AVALON_ADDR_WIDTH{1'b0}};
        m_writedata <= {AVALON_DATA_WIDTH{1'b0}};
        write_head <= {FIFO_PTR_WIDTH{1'b0}};
        write_tail <= {FIFO_PTR_WIDTH{1'b0}};
        write_axi_beats_received <= 8'b0;
        write_axi_total_beats <= 8'b0;
        write_axi_last_beat_received <= 1'b0;
        write_response_pending <= 1'b0;
        write_valid_reg <= 1'b0;
        write_resp_id <= {AXI_ID_WIDTH{1'b0}};
        write_avalon_total_subwrites <= 0;
        write_avalon_subwrites_remaining <= 0;
        write_base_addr <= {AXI_ADDR_WIDTH{1'b0}};
        write_base_addr_aligned <= {AXI_ADDR_WIDTH{1'b0}};
        
        for (i = 0; i < 4; i = i + 1) read_hold[i] <= 16'b0;
        write_timeout <= 16'b0;
        read_timeout <= 16'b0;
    end else begin
        case (state)
            // ========================================
            // IDLE: Wait for transactions
            // ========================================
            IDLE: begin
                s_awready <= 1'b1;
                s_arready <= 1'b1;
                s_wready <= 1'b0;
                m_write <= 1'b0;
                m_read <= 1'b0;
                write_valid_reg <= 1'b0;
                write_timeout <= 16'b0;
                
                if (write_response_pending) begin
                    state <= WRITE_RESP;
                end
                // Write request
                else if (s_awvalid && s_awready) begin
                    write_axi_id <= s_awid;
                    write_base_addr <= s_awaddr;
                    write_base_addr_aligned <= {s_awaddr[AXI_ADDR_WIDTH-1:3], 3'b0};
                    write_axi_total_beats <= s_awlen + 1;
                    write_axi_beats_received <= 8'b0;
                    write_axi_last_beat_received <= 1'b0;
                    write_resp_id <= s_awid;
                    write_avalon_total_subwrites <= (s_awlen + 1) * AVALON_PER_AXI;
                    write_avalon_subwrites_remaining <= (s_awlen + 1) * AVALON_PER_AXI;
                    
                    s_awready <= 1'b0;
                    s_arready <= 1'b0;
                    
                    $display("  [BRIDGE] Write request: ID=%0d addr=%h len=%0d", 
                             s_awid, s_awaddr, s_awlen);
                    $display("  [BRIDGE]   AXI beats: %0d, Avalon sub-writes: %0d", 
                             s_awlen+1, (s_awlen+1)*AVALON_PER_AXI);
                    
                    state <= WRITE_RECV;
                end
                // Read request
                else if (s_arvalid && s_arready) begin
                    read_base_addr <= s_araddr;
                    read_base_addr_aligned <= {s_araddr[AXI_ADDR_WIDTH-1:3], 3'b0};
                    read_id <= s_arid;
                    read_len <= s_arlen;
                    read_beat_count <= 8'b0;
                    read_sub_count <= 3'b0;
                    s_arready <= 1'b0;
                    s_awready <= 1'b0;
                    $display("  [BRIDGE] Read request: ID=%0d addr=%h len=%0d", 
                             s_arid, s_araddr, s_arlen);
                    state <= READ_ADDR;
                end
            end
            
            // ========================================
            // WRITE_RECV: Receive ALL AXI write beats into FIFO
            // ========================================
            WRITE_RECV: begin
                s_awready <= 1'b0;
                s_arready <= 1'b0;
                s_wready <= 1'b1;
                
                if (s_wvalid && s_wready) begin
                    // Check FIFO space for all sub-writes of this beat
                    if (write_fifo_full || 
                        (write_tail + AVALON_PER_AXI) % FIFO_DEPTH == write_head) begin
                        s_wready <= 1'b0;
                        $display("  [BRIDGE] FIFO full, stalling");
                    end else begin
                        $display("  [BRIDGE] Receiving AXI beat %0d of %0d: data=%h", 
                                 write_axi_beats_received, write_axi_total_beats, s_wdata);
                        
                        // Store all sub-beats for this AXI beat
                        for (i = 0; i < AVALON_PER_AXI; i = i + 1) begin
                            fifo_idx = (write_tail + i) % FIFO_DEPTH;
                            
                            write_id_fifo[fifo_idx] <= write_axi_id;
                            write_data_fifo[fifo_idx] <= s_wdata[i * AVALON_DATA_WIDTH +: AVALON_DATA_WIDTH];
                            write_strb_fifo[fifo_idx] <= s_wstrb[i * STRB_PER_AVALON +: STRB_PER_AVALON];
                            
                            // Calculate byte address for this 16-bit word using saved base address
                            beat_offset = write_axi_beats_received * BYTES_PER_AXI;
                            word_offset = i * BYTES_PER_AVALON;
                            word_byte_addr = write_base_addr_aligned + beat_offset + word_offset;
                            write_addr_fifo[fifo_idx] <= $unsigned(word_byte_addr >> 1);
                            
                            $display("  [BRIDGE]   Sub-write[%0d]: addr=%h data=%h", 
                                     i, $unsigned(word_byte_addr >> 1),
                                     s_wdata[i * AVALON_DATA_WIDTH +: AVALON_DATA_WIDTH]);
                        end
                        
                        write_tail <= (write_tail + AVALON_PER_AXI) % FIFO_DEPTH;
                        write_axi_beats_received <= write_axi_beats_received + 1;
                        
                        // Check if all beats received
                        if (s_wlast || write_axi_beats_received + 1 == write_axi_total_beats) begin
                            write_axi_last_beat_received <= 1'b1;
                            $display("  [BRIDGE] All %0d AXI beats received, starting flush", 
                                     write_axi_total_beats);
                            s_wready <= 1'b0;
                            state <= WRITE_FLUSH;
                        end
                    end
                end
            end
            
            // ========================================
            // WRITE_FLUSH: Flush FIFO to Avalon
            // ========================================
            WRITE_FLUSH: begin
                s_awready <= 1'b0;
                s_arready <= 1'b0;
                s_wready <= 1'b0;
                
                // Load next sub-write ONLY when:
                // 1. FIFO not empty
                // 2. No pending write request (write_valid_reg is 0)
                // 3. No active Avalon write (m_write is 0)
                if (!write_fifo_empty && !write_valid_reg && !m_write) begin
                    write_addr_reg <= write_addr_fifo[write_head];
                    write_data_reg <= write_data_fifo[write_head];
                    write_be_reg <= write_strb_fifo[write_head];
                    write_valid_reg <= 1'b1;
                    $display("  [BRIDGE] FLUSH: loading sub-write addr=%h data=%h, remaining=%0d", 
                             write_addr_fifo[write_head], write_data_fifo[write_head], 
                             write_avalon_subwrites_remaining);
                end
                
                // Drive Avalon write signals
                if (write_valid_reg && !m_write) begin
                    m_address <= write_addr_reg;
                    m_writedata <= write_data_reg;
                    m_byteenable <= write_be_reg;
                    m_write <= 1'b1;
                    write_valid_reg <= 1'b0;  // Clear only after starting the write
                end
                
                // Wait for Avalon write to complete
                if (!m_waitrequest && m_write) begin
                    m_write <= 1'b0;
                    write_timeout <= 16'b0;
                    
                    // Update FIFO head pointer
                    write_head <= (write_head + 1) % FIFO_DEPTH;
                    
                    // Check if this is the last sub-write
                    if (write_avalon_subwrites_remaining == 1) begin
                        write_avalon_subwrites_remaining <= 0;
                        $display("  [BRIDGE] All %0d Avalon sub-writes completed", 
                                 write_avalon_total_subwrites);
                        write_response_pending <= 1'b1;
                        state <= WRITE_RESP;
                    end else begin
                        write_avalon_subwrites_remaining <= write_avalon_subwrites_remaining - 1;
                    end
                end else begin
                    // Timeout protection
                    if (write_timeout == 16'hFFFF) begin
                        $display("  [BRIDGE] Write flush timeout!");
                        m_write <= 1'b0;
                        write_response_pending <= 1'b1;
                        state <= WRITE_RESP;
                        write_timeout <= 16'b0;
                    end else begin
                        write_timeout <= write_timeout + 1;
                    end
                end
            end
            
            // ========================================
            // WRITE_RESP: Send write response to AXI
            // ========================================
            WRITE_RESP: begin
                s_awready <= 1'b0;
                s_arready <= 1'b0;
                
                if (!s_bvalid) begin
                    s_bresp <= 2'b00;           // OKAY response
                    s_bid <= write_resp_id;
                    s_bvalid <= 1'b1;
                    $display("  [BRIDGE] Sending write response: ID=%0d", write_resp_id);
                end
                
                if (s_bvalid && s_bready) begin
                    s_bvalid <= 1'b0;
                    s_awready <= 1'b1;
                    s_arready <= 1'b1;
                    write_response_pending <= 1'b0;
                    state <= IDLE;
                end
                
                if (write_timeout == 16'hFFFF) begin
                    $display("  [BRIDGE] Write response timeout!");
                    s_bvalid <= 1'b0;
                    write_response_pending <= 1'b0;
                    s_awready <= 1'b1;
                    s_arready <= 1'b1;
                    state <= IDLE;
                    write_timeout <= 16'b0;
                end else begin
                    write_timeout <= write_timeout + 1;
                end
            end
            
            // ========================================
            // READ_ADDR: Setup read address for Avalon
            // ========================================
            READ_ADDR: begin
                s_awready <= 1'b0;
                s_arready <= 1'b0;
                
                beat_offset = read_beat_count * BYTES_PER_AXI;
                word_offset = read_sub_count * BYTES_PER_AVALON;
                word_byte_addr = read_base_addr_aligned + beat_offset + word_offset;
                
                read_addr_reg <= $unsigned(word_byte_addr >> 1);
                read_sub_reg <= read_sub_count;
                read_valid_reg <= 1'b1;
                
                state <= READ_AVALON;
            end
            
            // ========================================
            // READ_AVALON: Perform Avalon read transfers
            // ========================================
            READ_AVALON: begin
                s_awready <= 1'b0;
                s_arready <= 1'b0;
                
                if (read_valid_reg) begin
                    m_address <= read_addr_reg;
                    m_read <= 1'b1;
                    read_valid_reg <= 1'b0;
                end
                
                if (!m_waitrequest && m_read) begin
                    m_read <= 1'b0;
                end
                
                if (m_readdatavalid) begin
                    read_hold[read_sub_reg] <= m_readdata;
                    
                    if (read_sub_reg == AVALON_PER_AXI - 1) begin
                        state <= READ_COMBINE;
                    end else begin
                        read_sub_count <= read_sub_count + 1;
                        state <= READ_ADDR;
                    end
                    read_timeout <= 16'b0;
                end else begin
                    if (read_timeout == 16'hFFFF) begin
                        $display("  [BRIDGE] Read timeout!");
                        m_read <= 1'b0;
                        state <= IDLE;
                        read_timeout <= 16'b0;
                    end else begin
                        read_timeout <= read_timeout + 1;
                    end
                end
            end
            
            // ========================================
            // READ_COMBINE: Combine 16-bit reads into 64-bit AXI data
            // ========================================
            READ_COMBINE: begin
                s_awready <= 1'b0;
                s_arready <= 1'b0;
                
                // Little-endian: hold[0] is LSB, hold[3] is MSB
                s_rdata <= {read_hold[3], read_hold[2], read_hold[1], read_hold[0]};
                s_rid <= read_id;
                s_rvalid <= 1'b1;
                
                if (read_beat_count == read_len) begin
                    s_rlast <= 1'b1;
                end
                
                read_beat_count <= read_beat_count + 1;
                read_sub_count <= 3'b0;
                state <= READ_DATA;
                read_timeout <= 16'b0;
            end
            
            // ========================================
            // READ_DATA: Send data to AXI read channel
            // ========================================
            READ_DATA: begin
                s_awready <= 1'b0;
                s_arready <= 1'b0;
                
                if (s_rready) begin
                    s_rvalid <= 1'b0;
                    s_rlast <= 1'b0;
                    
                    if (read_beat_count > read_len) begin
                        // Burst complete
                        s_arready <= 1'b1;
                        s_awready <= 1'b1;
                        state <= IDLE;
                    end else begin
                        // Continue with next beat
                        read_sub_count <= 3'b0;
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
        m_burstcount <= 1'b1;  // Single-beat bursts on Avalon
        s_rresp <= 2'b00;      // OKAY response for reads
    end
end

// ====================================================
// Debug monitor (synthesis ignores)
// ====================================================
// synthesis translate_off
always @(posedge clk) begin
    if (m_write) begin
        $display("  [BRIDGE] Avalon Write: addr=0x%h data=0x%h be=%b", 
                 m_address, m_writedata, m_byteenable);
    end
    if (m_read) begin
        $display("  [BRIDGE] Avalon Read: addr=0x%h", m_address);
    end
    if (write_fifo_full) begin
        $display("  [BRIDGE] Warning: Write FIFO full");
    end
end
// synthesis translate_on

endmodule
