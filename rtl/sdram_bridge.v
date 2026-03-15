// AXI to Avalon Bridge
// Converts AXI4 bursts to 16-bit Avalon accesses
// AXI Slave interface (64-bit) to Avalon Master interface (16-bit)

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

// ====================================================
// States
// ====================================================
localparam IDLE         = 4'd0;
localparam WRITE_DATA   = 4'd2;
localparam WRITE_AVALON = 4'd4;
localparam WRITE_RESP   = 4'd5;
localparam READ_AVALON  = 4'd7;
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
reg                         write_burst_active;
reg                         write_response_pending;
reg                         write_resp_sent;

// Read tracking
reg [AXI_ADDR_WIDTH-1:0]    read_base_addr;
reg [7:0]                   read_len;
reg [7:0]                   read_beat_count;
reg [2:0]                   read_sub_count;
reg [15:0]                  read_hold0;
reg [15:0]                  read_hold1;
reg [15:0]                  read_hold2;
reg [15:0]                  read_hold3;
reg                         read_burst_active;

// Loop variables
integer i;
integer fifo_idx;

// Timeout counters (synthesis ignores these if not used in logic)
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
        write_head <= 0;
        write_tail <= 0;
        write_beat_count <= 0;
        write_burst_active <= 1'b0;
        write_response_pending <= 1'b0;
        write_resp_sent <= 1'b0;
        read_beat_count <= 0;
        read_sub_count <= 0;
        read_burst_active <= 1'b0;
        read_hold0 <= 16'b0;
        read_hold1 <= 16'b0;
        read_hold2 <= 16'b0;
        read_hold3 <= 16'b0;
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
                
                // If there's a pending write response not yet sent, enter WRITE_RESP
                if (write_response_pending && !write_resp_sent) begin
                    state <= WRITE_RESP;
                end
                // Check for write request
                else if (s_awvalid && s_awready) begin
                    // Write request
                    write_base_addr <= s_awaddr;
                    write_len <= s_awlen;
                    write_beat_count <= 0;
                    write_burst_active <= 1'b1;
                    write_resp_sent <= 1'b0;
                    s_awready <= 1'b0;
                    s_arready <= 1'b0;
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
                    state <= READ_AVALON;
                end
            end
            
            // ========================================
            // WRITE: Receive AXI data
            // ========================================
            WRITE_DATA: begin
                s_wready <= 1'b1;
                
                if (s_wvalid && s_wready) begin
                    // Decompose AXI data into 16-bit words
                    for (i = 0; i < BEATS_PER_AXI; i = i + 1) begin
                        fifo_idx = (write_tail + i) % FIFO_DEPTH;
                        
                        // Store 16-bit word
                        write_data_fifo[fifo_idx] <= 
                            s_wdata[i * AVALON_DATA_WIDTH +: AVALON_DATA_WIDTH];
                        
                        // Store byte enable
                        write_strb_fifo[fifo_idx] <= 
                            s_wstrb[i * (AVALON_DATA_WIDTH/8) +: (AVALON_DATA_WIDTH/8)];
                        
                        // Calculate address (convert byte address to word address)
                        write_addr_fifo[fifo_idx] <= 
                            (write_base_addr >> 1) + 
                            (write_beat_count * BEATS_PER_AXI) + i;
                    end
                    
                    write_tail <= write_tail + BEATS_PER_AXI;
                    write_beat_count <= write_beat_count + 1;
                    
                    if (s_wlast) begin
                        s_wready <= 1'b0;
                        state <= WRITE_AVALON;
                    end
                end
            end
            
            // ========================================
            // WRITE: Send to Avalon
            // ========================================
            WRITE_AVALON: begin
                if (!write_fifo_empty) begin
                    // Get next write from FIFO
                    m_address <= write_addr_fifo[write_head];
                    m_writedata <= write_data_fifo[write_head];
                    m_byteenable <= write_strb_fifo[write_head];
                    m_write <= 1'b1;
                    
                    if (!m_waitrequest) begin
                        m_write <= 1'b0;
                        write_head <= write_head + 1;
                        write_timeout <= 0;
                        
                        // Check if all writes are done
                        if (write_head == (write_tail - 1)) begin
                            write_response_pending <= 1'b1;
                            state <= IDLE;  // Go back to IDLE
                        end
                    end else begin
                        // Waitrequest active, count timeout
                        if (write_timeout == 16'hFFFF) begin
                            // Timeout - force completion
                            write_response_pending <= 1'b1;
                            state <= IDLE;
                            write_timeout <= 0;
                        end else begin
                            write_timeout <= write_timeout + 1;
                        end
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
                end
                
                if (s_bvalid && s_bready) begin
                    s_bvalid <= 1'b0;
                    s_awready <= 1'b1;
                    s_arready <= 1'b1;
                    write_burst_active <= 1'b0;
                    write_response_pending <= 1'b0;
                    write_resp_sent <= 1'b0;
                    state <= IDLE;
                end
                
                if (write_timeout == 16'hFFFF) begin
                    // Timeout - force clear
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
            // READ: Avalon reads - Synthesizable version
            // ========================================
            READ_AVALON: begin
                // Calculate current address
                m_address <= (read_base_addr >> 1) + 
                             (read_beat_count * BEATS_PER_AXI) + read_sub_count;
                m_read <= 1'b1;
                
                if (!m_waitrequest) begin
                    m_read <= 1'b0;
                end
                
                if (m_readdatavalid) begin
                    // Store data based on sub_count
                    case (read_sub_count)
                        2'd0: read_hold0 <= m_readdata;
                        2'd1: read_hold1 <= m_readdata;
                        2'd2: read_hold2 <= m_readdata;
                        2'd3: read_hold3 <= m_readdata;
                    endcase
                    
                    // Check if this is the last sub-beat
                    if (read_sub_count == BEATS_PER_AXI - 1) begin
                        // Combine 64-bit data using the newly received data
                        case (read_sub_count)
                            2'd0: s_rdata <= {read_hold3, read_hold2, read_hold1, m_readdata};
                            2'd1: s_rdata <= {read_hold3, read_hold2, m_readdata, read_hold0};
                            2'd2: s_rdata <= {read_hold3, m_readdata, read_hold1, read_hold0};
                            2'd3: s_rdata <= {m_readdata, read_hold2, read_hold1, read_hold0};
                        endcase
                        
                        s_rvalid <= 1'b1;
                        
                        if (read_beat_count == read_len) begin
                            s_rlast <= 1'b1;
                        end
                        
                        read_beat_count <= read_beat_count + 1;
                        read_sub_count <= 0;
                        state <= READ_DATA;
                    end else begin
                        read_sub_count <= read_sub_count + 1;
                    end
                    read_timeout <= 0;
                end else begin
                    if (read_timeout == 16'hFFFF) begin
                        // Timeout
                        state <= IDLE;
                        read_timeout <= 0;
                    end else begin
                        read_timeout <= read_timeout + 1;
                    end
                end
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
                        state <= READ_AVALON;
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

endmodule
