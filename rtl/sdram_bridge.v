// AXI to Avalon Bridge with ID support
// Converts AXI4 bursts to 16-bit Avalon accesses
// AXI Slave interface (64-bit) to Avalon Master interface (16-bit)

module axi_burst_master_to_avalon16 #(
    parameter AXI_ADDR_WIDTH = 32,
    parameter AXI_DATA_WIDTH = 64,
    parameter AVALON_ADDR_WIDTH = 24,
    parameter AVALON_DATA_WIDTH = 16,
    parameter AXI_MAX_BURST = 16,
    parameter FIFO_DEPTH = 32,
    parameter AXI_ID_WIDTH = 4
) (
    // Clock and reset
    input  wire                        clk,
    input  wire                        reset_n,
    
    // AXI4 Slave Interface - Write Address Channel with ID
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
    
    // Write Response Channel with ID
    output reg  [AXI_ID_WIDTH-1:0]     s_bid,
    output reg  [1:0]                  s_bresp,
    output reg                         s_bvalid,
    input  wire                        s_bready,
    
    // Read Address Channel with ID
    input  wire [AXI_ID_WIDTH-1:0]     s_arid,
    input  wire [AXI_ADDR_WIDTH-1:0]   s_araddr,
    input  wire [7:0]                  s_arlen,
    input  wire [2:0]                  s_arsize,
    input  wire [1:0]                  s_arburst,
    input  wire                        s_arvalid,
    output reg                         s_arready,
    
    // Read Data Channel with ID
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
localparam BYTES_PER_AVALON = AVALON_DATA_WIDTH / 8;      // 2 bytes per Avalon word
localparam BYTES_PER_AXI    = AXI_DATA_WIDTH / 8;         // 8 bytes per AXI beat
localparam AVALON_PER_AXI   = BYTES_PER_AXI / BYTES_PER_AVALON;  // 4 Avalon words per AXI beat
localparam STRB_PER_AVALON  = AVALON_DATA_WIDTH / 8;      // 2 strobe bits per Avalon word
localparam FIFO_PTR_WIDTH   = $clog2(FIFO_DEPTH);

// ====================================================
// States
// ====================================================
localparam IDLE         = 4'd0;
localparam WRITE_DATA   = 4'd2;
localparam WRITE_ADDR   = 4'd3;
localparam WRITE_AVALON = 4'd4;
localparam WRITE_RESP   = 4'd5;
localparam READ_ADDR    = 4'd6;
localparam READ_AVALON  = 4'd7;
localparam READ_COMBINE = 4'd8;
localparam READ_DATA    = 4'd9;

// ====================================================
// Internal Registers and Wires
// ====================================================
reg [3:0] state;

// Write FIFO - store data, address, strobe, and ID
reg [AXI_ID_WIDTH-1:0]       write_id_fifo [0:FIFO_DEPTH-1];
reg [AVALON_ADDR_WIDTH-1:0]  write_addr_fifo [0:FIFO_DEPTH-1];
reg [AVALON_DATA_WIDTH-1:0]  write_data_fifo [0:FIFO_DEPTH-1];
reg [1:0]                    write_strb_fifo [0:FIFO_DEPTH-1];
reg [FIFO_PTR_WIDTH-1:0]     write_head;
reg [FIFO_PTR_WIDTH-1:0]     write_tail;
wire                         write_fifo_empty;
wire                         write_fifo_full;

// Write tracking
reg [AXI_ADDR_WIDTH-1:0]     write_base_addr;
reg [AXI_ADDR_WIDTH-1:0]     write_base_addr_aligned;  // 8-byte aligned base address
reg [AXI_ID_WIDTH-1:0]       write_id;
reg [7:0]                    write_len;
reg [7:0]                    write_beat_count;
reg [7:0]                    write_total_beats;
reg                          write_response_pending;
reg                          write_resp_sent;
reg [AXI_ID_WIDTH-1:0]       write_resp_id;
reg                          write_fifo_stall;

// Write pipeline registers
reg [AVALON_ADDR_WIDTH-1:0]  write_addr_reg;
reg [AVALON_DATA_WIDTH-1:0]  write_data_reg;
reg [1:0]                    write_be_reg;
reg                          write_valid_reg;

// Write burst tracking
reg [7:0]                    write_burst_remaining;    // Remaining sub-writes in current burst

// Read tracking
reg [AXI_ADDR_WIDTH-1:0]     read_base_addr;
reg [AXI_ADDR_WIDTH-1:0]     read_base_addr_aligned;   // 8-byte aligned base address
reg [AXI_ID_WIDTH-1:0]       read_id;
reg [7:0]                    read_len;
reg [7:0]                    read_beat_count;
reg [2:0]                    read_sub_count;
reg [15:0]                   read_hold [0:3];

// Read address pipeline
reg [AVALON_ADDR_WIDTH-1:0]  read_addr_reg;
reg                          read_valid_reg;
reg [2:0]                    read_sub_reg;

// Loop variables and temporary registers
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
        write_beat_count <= 8'b0;
        write_total_beats <= 8'b0;
        write_response_pending <= 1'b0;
        write_resp_sent <= 1'b0;
        write_valid_reg <= 1'b0;
        write_addr_reg <= {AVALON_ADDR_WIDTH{1'b0}};
        write_data_reg <= {AVALON_DATA_WIDTH{1'b0}};
        write_be_reg <= 2'b0;
        write_resp_id <= {AXI_ID_WIDTH{1'b0}};
        write_fifo_stall <= 1'b0;
        write_base_addr_aligned <= {AXI_ADDR_WIDTH{1'b0}};
        write_burst_remaining <= 8'b0;
        read_beat_count <= 8'b0;
        read_sub_count <= 3'b0;
        read_addr_reg <= {AVALON_ADDR_WIDTH{1'b0}};
        read_valid_reg <= 1'b0;
        read_sub_reg <= 3'b0;
        read_base_addr_aligned <= {AXI_ADDR_WIDTH{1'b0}};
        word_byte_addr <= {AXI_ADDR_WIDTH{1'b0}};
        beat_offset <= {AXI_ADDR_WIDTH{1'b0}};
        word_offset <= {AXI_ADDR_WIDTH{1'b0}};
        for (i = 0; i < 4; i = i + 1) read_hold[i] <= 16'b0;
        write_timeout <= 16'b0;
        read_timeout <= 16'b0;
    end else begin
        case (state)
            // ========================================
            // IDLE: Wait for AXI transactions
            // ========================================
            IDLE: begin
                s_awready <= 1'b1;
                s_arready <= 1'b1;
                s_wready <= 1'b0;
                m_write <= 1'b0;
                m_read <= 1'b0;
                write_timeout <= 16'b0;
                read_timeout <= 16'b0;
                write_valid_reg <= 1'b0;
                write_fifo_stall <= 1'b0;
                
                // If there's a pending write response not yet sent, enter WRITE_RESP
                if (write_response_pending && !write_resp_sent) begin
                    state <= WRITE_RESP;
                end
                // Check for write request
                else if (s_awvalid && s_awready) begin
                    // Store write request information
                    write_base_addr <= s_awaddr;
                    // Align to 8-byte boundary for correct address calculation
                    write_base_addr_aligned <= {s_awaddr[AXI_ADDR_WIDTH-1:3], 3'b0};
                    write_id <= s_awid;
                    write_len <= s_awlen;
                    write_total_beats <= s_awlen + 1;
                    write_beat_count <= 8'b0;
                    write_resp_sent <= 1'b0;
                    write_burst_remaining <= AVALON_PER_AXI;  // Need to complete all sub-writes
                    s_awready <= 1'b0;
                    s_arready <= 1'b0;
                    $display("  [BRIDGE] Write request: ID=%0d addr=%h len=%0d", 
                             s_awid, s_awaddr, s_awlen);
                    state <= WRITE_DATA;
                end
                // Check for read request
                else if (s_arvalid && s_arready) begin
                    // Store read request information
                    read_base_addr <= s_araddr;
                    // Align to 8-byte boundary for correct address calculation
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
            // WRITE_DATA: Receive AXI write data beats
            // ========================================
            WRITE_DATA: begin
                // Block new read/write requests while processing write
                s_awready <= 1'b0;
                s_arready <= 1'b0;
                s_wready <= 1'b1;
                
                if (s_wvalid && s_wready) begin
                    // Check if FIFO has space for all sub-beats
                    if (write_fifo_full || (write_tail + AVALON_PER_AXI) % FIFO_DEPTH == write_head) begin
                        // FIFO full, stall the AXI write data channel
                        write_fifo_stall <= 1'b1;
                        s_wready <= 1'b0;
                        $display("  [BRIDGE] Write FIFO full, stalling");
                    end else begin
                        $display("  [BRIDGE] Write data received: beat=%0d data=%h", 
                                 write_beat_count, s_wdata);
                        
                        // Decompose 64-bit AXI data into 16-bit Avalon words
                        for (i = 0; i < AVALON_PER_AXI; i = i + 1) begin
                            fifo_idx = (write_tail + i) % FIFO_DEPTH;
                            
                            // Store ID (same for all sub-beats)
                            write_id_fifo[fifo_idx] <= write_id;
                            
                            // Extract 16-bit word from the 64-bit data
                            write_data_fifo[fifo_idx] <= s_wdata[i * AVALON_DATA_WIDTH +: AVALON_DATA_WIDTH];
                            
                            // Extract byte enables for this 16-bit word (2 bits per word)
                            write_strb_fifo[fifo_idx] <= s_wstrb[i * STRB_PER_AVALON +: STRB_PER_AVALON];
                            
                            // Calculate correct byte address for this 16-bit word
                            // Formula: byte_addr = aligned_base + (beat * 8) + (i * 2)
                            // Then convert to Avalon word address by shifting right by 1
                            beat_offset = write_beat_count * BYTES_PER_AXI;
                            word_offset = i * BYTES_PER_AVALON;
                            word_byte_addr = write_base_addr_aligned + beat_offset + word_offset;
                            
                            write_addr_fifo[fifo_idx] <= $unsigned(word_byte_addr >> 1);
                            
                            $display("  [BRIDGE]   FIFO[%0d]: ID=%0d byte_addr=%h word_addr=%h data=%h be=%b", 
                                     fifo_idx, write_id, word_byte_addr,
                                     $unsigned(word_byte_addr >> 1),
                                     s_wdata[i * AVALON_DATA_WIDTH +: AVALON_DATA_WIDTH],
                                     s_wstrb[i * STRB_PER_AVALON +: STRB_PER_AVALON]);
                        end
                        
                        // Update tail pointer
                        write_tail <= (write_tail + AVALON_PER_AXI) % FIFO_DEPTH;
                        write_beat_count <= write_beat_count + 1;
                        write_fifo_stall <= 1'b0;
                        
                        // On last beat, prepare for address phase
                        if (s_wlast) begin
                            write_resp_id <= write_id;
                            write_burst_remaining <= AVALON_PER_AXI;  // Reset for the coming burst
                            $display("  [BRIDGE] Last write data received, response ID=%0d", write_id);
                            s_wready <= 1'b0;
                            state <= WRITE_ADDR;
                        end
                    end
                end
            end
            
            // ========================================
            // WRITE_ADDR: Set up address and data for Avalon
            // ========================================
            WRITE_ADDR: begin
                // Block new read/write requests while processing write
                s_awready <= 1'b0;
                s_arready <= 1'b0;
                
                if (!write_fifo_empty) begin
                    // Set up address and data registers from FIFO
                    write_addr_reg <= write_addr_fifo[write_head];
                    write_data_reg <= write_data_fifo[write_head];
                    write_be_reg <= write_strb_fifo[write_head];
                    write_resp_id <= write_id_fifo[write_head];
                    write_valid_reg <= 1'b1;
                    
                    $display("  [BRIDGE] WRITE_ADDR: ID=%0d addr=%h data=%h be=%b", 
                             write_id_fifo[write_head],
                             write_addr_fifo[write_head], 
                             write_data_fifo[write_head], 
                             write_strb_fifo[write_head]);
                    
                    state <= WRITE_AVALON;
                end else begin
                    // FIFO empty, go back to IDLE
                    state <= IDLE;
                end
            end
            
            // ========================================
            // WRITE_AVALON: Perform Avalon write transfer
            // ========================================
            WRITE_AVALON: begin
                // Block new read/write requests while processing write
                s_awready <= 1'b0;
                s_arready <= 1'b0;
                
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
                
                // Wait for Avalon slave to accept the write
                if (!m_waitrequest && m_write) begin
                    $display("  [BRIDGE] Write accepted at time %t", $time);
                    m_write <= 1'b0;
                    
                    // Update head pointer to remove completed transfer from FIFO
                    write_head <= (write_head + 1) % FIFO_DEPTH;
                    write_timeout <= 16'b0;
                    
                    // Decrement remaining sub-writes in current burst
                    if (write_burst_remaining > 0) begin
                        write_burst_remaining <= write_burst_remaining - 1;
                    end
                    
                    // Check if all writes in the current burst are completed
                    if (write_burst_remaining == 1) begin
                        // This was the last sub-write of the burst
                        $display("  [BRIDGE] Write burst completed at time %t", $time);
                        write_response_pending <= 1'b1;
                        state <= WRITE_RESP;
                    end else if (((write_head + 1) % FIFO_DEPTH) == write_tail) begin
                        // FIFO empty, no more writes
                        state <= IDLE;
                    end else begin
                        // More writes to process in this burst
                        state <= WRITE_ADDR;
                    end
                end else begin
                    // Timeout protection for stuck writes
                    if (write_timeout == 16'hFFFF) begin
                        $display("  [BRIDGE] Write timeout! addr=%h", m_address);
                        write_response_pending <= 1'b1;
                        m_write <= 1'b0;
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
                // Block new read/write requests until response is sent
                s_awready <= 1'b0;
                s_arready <= 1'b0;
                
                if (!s_bvalid) begin
                    s_bresp <= 2'b00;           // OKAY response
                    s_bid <= write_resp_id;      // Return stored ID
                    s_bvalid <= 1'b1;
                    write_resp_sent <= 1'b1;
                    $display("  [BRIDGE] Sending write response: ID=%0d at time %t", 
                             write_resp_id, $time);
                end
                
                if (s_bvalid && s_bready) begin
                    $display("  [BRIDGE] Write response accepted: ID=%0d at time %t", 
                             s_bid, $time);
                    s_bvalid <= 1'b0;
                    // Re-enable new requests only after response is sent
                    s_awready <= 1'b1;
                    s_arready <= 1'b1;
                    write_response_pending <= 1'b0;
                    write_resp_sent <= 1'b0;
                    write_burst_remaining <= 8'b0;
                    state <= IDLE;
                end
                
                if (write_timeout == 16'hFFFF) begin
                    $display("  [BRIDGE] Write response timeout!");
                    s_bvalid <= 1'b0;
                    write_response_pending <= 1'b0;
                    write_resp_sent <= 1'b0;
                    s_awready <= 1'b1;
                    s_arready <= 1'b1;
                    write_burst_remaining <= 8'b0;
                    state <= IDLE;
                    write_timeout <= 16'b0;
                end else begin
                    write_timeout <= write_timeout + 1;
                end
            end
            
            // ========================================
            // READ_ADDR: Set up address for Avalon read
            // ========================================
            READ_ADDR: begin
                // Block new read/write requests while processing read
                s_awready <= 1'b0;
                s_arready <= 1'b0;
                
                // Calculate the byte address for this sub-read
                beat_offset = read_beat_count * BYTES_PER_AXI;
                word_offset = read_sub_count * BYTES_PER_AVALON;
                word_byte_addr = read_base_addr_aligned + beat_offset + word_offset;
                
                read_addr_reg <= $unsigned(word_byte_addr >> 1);
                read_sub_reg <= read_sub_count;
                read_valid_reg <= 1'b1;
                
                $display("  [BRIDGE] READ_ADDR: ID=%0d beat=%0d sub=%0d byte_addr=%h word_addr=%h", 
                         read_id, read_beat_count, read_sub_count, word_byte_addr,
                         $unsigned(word_byte_addr >> 1));
                
                state <= READ_AVALON;
            end
            
            // ========================================
            // READ_AVALON: Perform Avalon read transfers
            // ========================================
            READ_AVALON: begin
                // Block new read/write requests while processing read
                s_awready <= 1'b0;
                s_arready <= 1'b0;
                
                if (read_valid_reg) begin
                    m_address <= read_addr_reg;
                    m_read <= 1'b1;
                    read_valid_reg <= 1'b0;
                    $display("  [BRIDGE] READ_AVALON: asserting read at addr=%h", read_addr_reg);
                end
                
                if (!m_waitrequest && m_read) begin
                    m_read <= 1'b0;
                end
                
                if (m_readdatavalid) begin
                    $display("  [BRIDGE] Read data received: sub=%0d data=%h", read_sub_reg, m_readdata);
                    
                    // Store data in order of arrival
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
                // Block new read/write requests while combining data
                s_awready <= 1'b0;
                s_arready <= 1'b0;
                
                // Little-endian: hold[0] is LSB, hold[3] is MSB
                s_rdata <= {read_hold[3], read_hold[2], read_hold[1], read_hold[0]};
                s_rid <= read_id;                       // Return stored ID
                s_rvalid <= 1'b1;
                
                $display("  [BRIDGE] Combining: ID=%0d {%h, %h, %h, %h} = %h", 
                         read_id,
                         read_hold[3], read_hold[2], read_hold[1], read_hold[0],
                         {read_hold[3], read_hold[2], read_hold[1], read_hold[0]});
                
                if (read_beat_count == read_len) begin
                    s_rlast <= 1'b1;
                    $display("  [BRIDGE] Last beat, ID=%0d", read_id);
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
                // Block new read/write requests while sending read data
                s_awready <= 1'b0;
                s_arready <= 1'b0;
                
                if (s_rready) begin
                    s_rvalid <= 1'b0;
                    s_rlast <= 1'b0;
                    
                    if (read_beat_count > read_len) begin
                        // Burst complete, re-enable new requests
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
        $display("  [BRIDGE] Avalon Write: addr=0x%h data=0x%h be=%b at time %t", 
                 m_address, m_writedata, m_byteenable, $time);
    end
    if (m_read) begin
        $display("  [BRIDGE] Avalon Read: addr=0x%h at time %t", 
                 m_address, $time);
    end
    // Monitor FIFO status
    if (write_fifo_full) begin
        $display("  [BRIDGE] Warning: Write FIFO full at time %t", $time);
    end
end
// synthesis translate_on

endmodule
