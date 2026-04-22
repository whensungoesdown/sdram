`timescale 1ns/1ps

module top_tb();

// ====================================================
// Parameters
// ====================================================
localparam CLK_PERIOD = 13.333;  // 75MHz = 13.333ns
localparam AXI_ADDR_WIDTH = 32;
localparam AXI_DATA_WIDTH = 64;
localparam AVALON_ADDR_WIDTH = 24;
localparam AVALON_DATA_WIDTH = 16;
localparam AXI_ID_WIDTH = 4;

// SDRAM geometry for W9825G6KH
localparam SDRAM_ROW_BITS = 13;
localparam SDRAM_COL_BITS = 9;
localparam SDRAM_BANK_BITS = 2;
localparam SDRAM_ROW_SIZE = (1 << SDRAM_COL_BITS) * 2;  // 512 * 2 = 1024 bytes per row

// ====================================================
// Testbench Signals
// ====================================================
reg                                 clk;
reg                                 reset_n;

// AXI Master Interface (from test stimulus)
reg  [AXI_ID_WIDTH-1:0]             axi_awid;
reg  [AXI_ADDR_WIDTH-1:0]           axi_awaddr;
reg  [7:0]                          axi_awlen;
reg  [2:0]                          axi_awsize;
reg  [1:0]                          axi_awburst;
reg                                 axi_awvalid;
wire                                axi_awready;

reg  [AXI_DATA_WIDTH-1:0]           axi_wdata;
reg  [AXI_DATA_WIDTH/8-1:0]         axi_wstrb;
reg                                 axi_wlast;
reg                                 axi_wvalid;
wire                                axi_wready;

wire [AXI_ID_WIDTH-1:0]             axi_bid;
wire [1:0]                          axi_bresp;
wire                                axi_bvalid;
reg                                 axi_bready;

reg  [AXI_ID_WIDTH-1:0]             axi_arid;
reg  [AXI_ADDR_WIDTH-1:0]           axi_araddr;
reg  [7:0]                          axi_arlen;
reg  [2:0]                          axi_arsize;
reg  [1:0]                          axi_arburst;
reg                                 axi_arvalid;
wire                                axi_arready;

wire [AXI_ID_WIDTH-1:0]             axi_rid;
wire [AXI_DATA_WIDTH-1:0]           axi_rdata;
wire [1:0]                          axi_rresp;
wire                                axi_rlast;
wire                                axi_rvalid;
reg                                 axi_rready;

// SDRAM Physical Interface
wire [15:0]                         sdram_dq;
wire [12:0]                         sdram_addr;
wire [1:0]                          sdram_ba;
wire                                sdram_cas_n;
wire                                sdram_ras_n;
wire                                sdram_we_n;
wire                                sdram_cs_n;
wire                                sdram_cke;
wire [1:0]                          sdram_dqm;

// Test control
integer i, j, k;
reg [63:0] expected;
reg [31:0] test_counter;
reg        test_passed;
reg        all_tests_passed;

// Timeout counter
reg [31:0] timeout_counter;
reg        response_received;

// ====================================================
// Internal signals between Bridge and SDRAM Controller
// ====================================================
wire [23:0]                         avm_address;
wire                                avm_write;
wire                                avm_read;
wire [15:0]                         avm_writedata;
wire [1:0]                          avm_byteenable;
wire                                avm_burstcount;

wire [15:0]                         avs_readdata;
wire                                avs_waitrequest;
wire                                avs_readdatavalid;

// ====================================================
// Function to convert AXI byte address to SDRAM row/col/bank
// ====================================================
function [31:0] get_row;
    input [31:0] byte_addr;
    begin
        get_row = (byte_addr >> 10) & 13'h1FFF;
    end
endfunction

function [31:0] get_col;
    input [31:0] byte_addr;
    begin
        get_col = (byte_addr >> 1) & 9'h1FF;
    end
endfunction

function [31:0] get_bank;
    input [31:0] byte_addr;
    begin
        get_bank = {byte_addr[23], byte_addr[9]};
    end
endfunction

// ====================================================
// Function to decode SDRAM command
// ====================================================
function [31:0] decode_sdram_cmd;
    input ras, cas, we;
    begin
        if (!ras && !cas && we) decode_sdram_cmd = "ACTV";
        else if (ras && !cas && we) decode_sdram_cmd = "READ";
        else if (ras && !cas && !we) decode_sdram_cmd = "WRIT";
        else if (!ras && cas && we) decode_sdram_cmd = "PRE ";
        else if (!ras && !cas && !we) decode_sdram_cmd = "REFR";
        else if (ras && cas && !we) decode_sdram_cmd = "MRS ";
        else decode_sdram_cmd = "NOP ";
    end
endfunction

// ====================================================
// Instantiate AXI to Avalon Bridge
// ====================================================
axi_burst_master_to_avalon16 #(
    .AXI_ADDR_WIDTH(AXI_ADDR_WIDTH),
    .AXI_DATA_WIDTH(AXI_DATA_WIDTH),
    .AVALON_ADDR_WIDTH(AVALON_ADDR_WIDTH),
    .AVALON_DATA_WIDTH(AVALON_DATA_WIDTH),
    .AXI_MAX_BURST(16),
    .FIFO_DEPTH(32)
) u_axi_bridge (
    .clk            (clk),
    .reset_n        (reset_n),
    
    // AXI Slave Interface
    .s_awid         (axi_awid),
    .s_awaddr       (axi_awaddr),
    .s_awlen        (axi_awlen),
    .s_awsize       (axi_awsize),
    .s_awburst      (axi_awburst),
    .s_awvalid      (axi_awvalid),
    .s_awready      (axi_awready),
    
    .s_wdata        (axi_wdata),
    .s_wstrb        (axi_wstrb),
    .s_wlast        (axi_wlast),
    .s_wvalid       (axi_wvalid),
    .s_wready       (axi_wready),
    
    .s_bid          (axi_bid),
    .s_bresp        (axi_bresp),
    .s_bvalid       (axi_bvalid),
    .s_bready       (axi_bready),
    
    .s_arid         (axi_arid),
    .s_araddr       (axi_araddr),
    .s_arlen        (axi_arlen),
    .s_arsize       (axi_arsize),
    .s_arburst      (axi_arburst),
    .s_arvalid      (axi_arvalid),
    .s_arready      (axi_arready),
    
    .s_rid          (axi_rid),
    .s_rdata        (axi_rdata),
    .s_rresp        (axi_rresp),
    .s_rlast        (axi_rlast),
    .s_rvalid       (axi_rvalid),
    .s_rready       (axi_rready),
    
    // Avalon Master Interface
    .m_address      (avm_address),
    .m_write        (avm_write),
    .m_read         (avm_read),
    .m_writedata    (avm_writedata),
    .m_readdata     (avs_readdata),
    .m_byteenable   (avm_byteenable),
    .m_waitrequest  (avs_waitrequest),
    .m_readdatavalid(avs_readdatavalid),
    .m_burstcount   (avm_burstcount)
);

// ====================================================
// Instantiate SDRAM Controller
// ====================================================
sdram_controller u_sdram_ctrl (
    .az_addr        (avm_address),
    .az_be_n        (~avm_byteenable),
    .az_cs          (avm_write | avm_read),
    .az_data        (avm_writedata),
    .az_rd_n        (~avm_read),
    .az_wr_n        (~avm_write),
    .clk            (clk),
    .reset_n        (reset_n),
    
    .za_data        (avs_readdata),
    .za_valid       (avs_readdatavalid),
    .za_waitrequest (avs_waitrequest),
    
    .zs_addr        (sdram_addr),
    .zs_ba          (sdram_ba),
    .zs_cas_n       (sdram_cas_n),
    .zs_cke         (sdram_cke),
    .zs_cs_n        (sdram_cs_n),
    .zs_dq          (sdram_dq),
    .zs_dqm         (sdram_dqm),
    .zs_ras_n       (sdram_ras_n),
    .zs_we_n        (sdram_we_n)
);

// ====================================================
// Instantiate SDRAM Test Component
// ====================================================
sdram_controller_test_component u_sdram_model (
    .clk            (clk),
    .zs_addr        (sdram_addr),
    .zs_ba          (sdram_ba),
    .zs_cas_n       (sdram_cas_n),
    .zs_cke         (sdram_cke),
    .zs_cs_n        (sdram_cs_n),
    .zs_dqm         (sdram_dqm),
    .zs_ras_n       (sdram_ras_n),
    .zs_we_n        (sdram_we_n),
    .zs_dq          (sdram_dq)
);

// ====================================================
// SDRAM Initialization Monitor
// ====================================================
reg init_monitor_done;
integer init_wait_count;
reg init_timeout;

always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        init_monitor_done <= 0;
        init_wait_count <= 0;
        init_timeout <= 0;
    end else if (!init_monitor_done) begin
        init_wait_count <= init_wait_count + 1;
        
        // Wait about 200us then assume initialization is done
        if (init_wait_count > 15000) begin
            init_monitor_done <= 1;
            init_timeout <= 1;
        end
        
        // Check if SDRAM has completed initialization
        if (u_sdram_ctrl.init_done) begin
            init_monitor_done <= 1;
            init_timeout <= 0;
        end
    end
end

// ====================================================
// Clock Generation
// ====================================================
initial begin
    clk = 0;
    forever #(CLK_PERIOD/2) clk = ~clk;
end

// ====================================================
// Timeout Monitor
// ====================================================
always @(posedge clk) begin
    if (reset_n) begin
        timeout_counter <= timeout_counter + 1;
        if (timeout_counter > 2000000) begin
            $display("========================================");
            $display("ERROR: Global simulation timeout at time %t", $time);
            $display("TEST FAILED: Simulation timeout");
            $display("========================================");
            $finish;
        end
    end else begin
        timeout_counter <= 0;
    end
end

// ====================================================
// Reset and Test Sequence
// ====================================================
initial begin
    // Initialize all signals
    reset_n = 0;
    axi_awid = 0;
    axi_awaddr = 0;
    axi_awlen = 0;
    axi_awsize = 3'b011;  // 8 bytes
    axi_awburst = 2'b01;  // INCR
    axi_awvalid = 0;
    axi_wdata = 0;
    axi_wstrb = 8'hFF;
    axi_wlast = 0;
    axi_wvalid = 0;
    axi_bready = 1;
    axi_arid = 0;
    axi_araddr = 0;
    axi_arlen = 0;
    axi_arsize = 3'b011;
    axi_arburst = 2'b01;
    axi_arvalid = 0;
    axi_rready = 1;
    
    all_tests_passed = 1;
    test_counter = 0;
    
    $display("========================================");
    $display("SDRAM Controller Top-Level Testbench");
    $display("AXI Bridge -> SDRAM Controller -> SDRAM Model");
    $display("Clock: 75MHz (%.3f ns)", CLK_PERIOD);
    $display("SDRAM: W9825G6KH (32MB, 8192 rows, 512 columns, 4 banks)");
    $display("========================================");
    
    // Reset sequence
    $display("Applying reset...");
    $display("========================================");
    #(CLK_PERIOD*20);
    reset_n = 1;
    
    $display("Waiting for SDRAM initialization (200us)...");
    wait(init_monitor_done);
    
    if (init_timeout) begin
        $display("WARNING: SDRAM initialization timed out, continuing anyway...");
    end else begin
        $display("SDRAM initialization completed at time %t", $time);
    end
    
    $display("Starting tests...");
    #(CLK_PERIOD*100);
    
    // Run tests
    // test_single_write and test_single_read should run together
    // test_single_write write data=0123456789abcdef to address=0000,
    // test_single_read reads it out
    test_single_write();
    test_single_read();
    test_write_read_verify();

    test_burst_write();
    test_burst_read();
    test_unaligned_access();
    test_multiple_rows();
    test_different_bank();
    
    // Test summary
    #(CLK_PERIOD*100);
    $display("========================================");
    $display("TEST SUMMARY");
    $display("========================================");
    if (all_tests_passed) begin
        $display("\nPASS!\n");
        $display("\033[0;32m");
        $display("**************************************************");
        $display("*                                                *");
        $display("*      * * *       *        * * *     * * *      *");
        $display("*      *    *     * *      *         *           *");
        $display("*      * * *     *   *      * * *     * * *      *");
        $display("*      *        * * * *          *         *     *");
        $display("*      *       *       *    * * *     * * *      *");
        $display("*                                                *");
        $display("**************************************************");
        $display("\n");
        $display("\033[0m");
    end else begin
        $display("\nFAIL!\n");
        $display("\033[0;31m");
        $display("**************************************************");
        $display("*                                                *");
        $display("*      * * *       *         ***      *          *");
        $display("*      *          * *         *       *          *");
        $display("*      * * *     *   *        *       *          *");
        $display("*      *        * * * *       *       *          *");
        $display("*      *       *       *     ***      * * *      *");
        $display("*                                                *");
        $display("**************************************************");
        $display("\n");
        $display("\033[0m");
    end
    $display("========================================");
    $finish;
end

// ====================================================
// Test 1: Single 64-bit Write
// ====================================================
task test_single_write;
    reg [31:0] timeout;
    begin
        test_counter = test_counter + 1;
        test_passed = 1;
        response_received = 0;
        
        $display("========================================");
        $display("Test %0d: Single 64-bit Write", test_counter);
        $display("========================================");
        
        axi_bready = 1'b1;
        
        @(posedge clk);
        axi_awaddr = 32'h0000_0000;
        axi_awlen = 0;
        axi_awvalid = 1;
        
        @(posedge clk);
        axi_awvalid = 0;
        
        // Wait for awready
        while (!axi_awready) @(posedge clk);
        @(posedge clk);
        
        @(posedge clk);
        axi_wdata = 64'h0123456789ABCDEF;
        axi_wstrb = 8'hFF;
        axi_wlast = 1;
        axi_wvalid = 1;
        
        @(posedge clk);
        axi_wvalid = 0;
        axi_wlast = 0;
        
        $display("  Waiting for write response...");
        
        // Wait for response with timeout
        timeout = 0;
        while (timeout < 200) begin
            @(posedge clk);
            if (axi_bvalid) begin
                response_received = 1;
                timeout = 200;
            end
            timeout = timeout + 1;
        end
        
        if (response_received) begin
            $display("  Write response received at time %t", $time);
            $display("  Write response: %b", axi_bresp);
            if (axi_bresp != 2'b00) begin
                $display("  ERROR: Write response not OKAY (got %b)", axi_bresp);
                test_passed = 0;
            end
        end else begin
            $display("  ERROR: Write response timeout!");
            test_passed = 0;
        end
        
        // Output result
        if (test_passed) begin
            $display("  ✓ TEST %0d: PASS", test_counter);
        end else begin
            $display("  ✗ TEST %0d: FAIL", test_counter);
            all_tests_passed = 0;
        end
        
        #(CLK_PERIOD*50);
    end
endtask

// ====================================================
// Test 2: Single 64-bit Read
// ====================================================
task test_single_read;
    reg [31:0] timeout;
    begin
        test_counter = test_counter + 1;
        test_passed = 1;
        
        $display("========================================");
        $display("Test %0d: Single 64-bit Read", test_counter);
        $display("========================================");
        
        #(CLK_PERIOD*50);
        
        axi_rready = 1'b1;
        
        @(posedge clk);
        axi_araddr = 32'h0000_0000;
        axi_arlen = 0;
        axi_arvalid = 1;
        
        @(posedge clk);
        axi_arvalid = 0;
        
        $display("  Waiting for read data...");
        
        // Wait for read data with timeout
        timeout = 0;
        while (timeout < 200) begin
            @(posedge clk);
            if (axi_rvalid) begin
                timeout = 200;
            end
            timeout = timeout + 1;
        end
        
        if (axi_rvalid) begin
            $display("  Read data received: %h at time %t", axi_rdata, $time);
            
            // Verify data
            expected = 64'h0123456789ABCDEF; // test1 writed to sdram
            if (axi_rdata !== expected) begin
                $display("  ERROR: Read data mismatch.");
                $display("    Expected: %h", expected);
                $display("    Got:      %h", axi_rdata);
                test_passed = 0;
            end else begin
                $display("    Data correct: %h", axi_rdata);
            end
        end else begin
            $display("  ERROR: No read data received!");
            test_passed = 0;
        end
        
        // Output result
        if (test_passed) begin
            $display("  ✓ TEST %0d: PASS", test_counter);
        end else begin
            $display("  ✗ TEST %0d: FAIL", test_counter);
            all_tests_passed = 0;
        end
        
        #(CLK_PERIOD*50);
    end
endtask

// ====================================================
// Test 3: Write then Read Verify
// ====================================================
task test_write_read_verify;
    reg [31:0] timeout;
    reg [63:0] write_data;
    reg [31:0] addr;
    begin
        test_counter = test_counter + 1;
        test_passed = 1;
        response_received = 0;
        
        $display("========================================");
        $display("Test %0d: Write then Read Verify", test_counter);
        $display("========================================");
        
        addr = 32'h1000_1000;
        write_data = 64'hDEADBEEFCAFEBABE;
        
        axi_bready = 1'b1;
        
        // Write
        @(posedge clk);
        axi_awaddr = addr;
        axi_awlen = 0;
        axi_awvalid = 1;
        
        @(posedge clk);
        axi_awvalid = 0;
        
        while (!axi_awready) @(posedge clk);
        @(posedge clk);
        
        @(posedge clk);
        axi_wdata = write_data;
        axi_wstrb = 8'hFF;
        axi_wlast = 1;
        axi_wvalid = 1;
        
        @(posedge clk);
        axi_wvalid = 0;
        axi_wlast = 0;
        
        $display("  Waiting for write response...");
        
        timeout = 0;
        while (timeout < 200) begin
            @(posedge clk);
            if (axi_bvalid) begin
                response_received = 1;
                timeout = 200;
            end
            timeout = timeout + 1;
        end
        
        if (!response_received) begin
            $display("  ERROR: Write response timeout!");
            test_passed = 0;
        end
        
        #(CLK_PERIOD*50);
        
        // Read
        @(posedge clk);
        axi_araddr = addr;
        axi_arlen = 0;
        axi_arvalid = 1;
        
        @(posedge clk);
        axi_arvalid = 0;
        
        $display("  Waiting for read data...");
        
        timeout = 0;
        while (timeout < 200) begin
            @(posedge clk);
            if (axi_rvalid) begin
                timeout = 200;
            end
            timeout = timeout + 1;
        end
        
        if (axi_rvalid) begin
            $display("  Read data received: %h at time %t", axi_rdata, $time);
            
            if (axi_rdata !== write_data) begin
                $display("  ERROR: Read data mismatch.");
                $display("    Expected: %h", write_data);
                $display("    Got:      %h", axi_rdata);
                test_passed = 0;
            end else begin
                $display("    Data correct: %h", axi_rdata);
            end
        end else begin
            $display("  ERROR: No read data received!");
            test_passed = 0;
        end
        
        // Output result
        if (test_passed) begin
            $display("  ✓ TEST %0d: PASS", test_counter);
        end else begin
            $display("  ✗ TEST %0d: FAIL", test_counter);
            all_tests_passed = 0;
        end
        
        #(CLK_PERIOD*50);
    end
endtask

// ====================================================
// Test 4: Burst Write of 4 Beats
// ====================================================
task test_burst_write;
    reg [31:0] timeout;
    reg [63:0] burst_data [0:3];
    begin
        test_counter = test_counter + 1;
        test_passed = 1;
        response_received = 0;
        
        $display("========================================");
        $display("Test %0d: Burst Write (4 beats)", test_counter);
        $display("========================================");
        
        #(CLK_PERIOD*50);
        
        axi_bready = 1'b1;
        
        // Prepare burst data
        burst_data[0] = 64'h1111111122222222;
        burst_data[1] = 64'h3333333344444444;
        burst_data[2] = 64'h5555555566666666;
        burst_data[3] = 64'h7777777788888888;
        
        // Start burst write
        @(posedge clk);
        axi_awaddr = 32'h1000_2000;
        axi_awlen = 3;  // 4 beats
        axi_awvalid = 1;
        
        @(posedge clk);
        axi_awvalid = 0;
        
        while (!axi_awready) @(posedge clk);
        @(posedge clk);
        
        // Send burst data
        for (i = 0; i < 4; i = i + 1) begin
            @(posedge clk);
            axi_wdata = burst_data[i];
            axi_wstrb = 8'hFF;
            axi_wlast = (i == 3);
            axi_wvalid = 1;
            
            @(posedge clk);
            axi_wvalid = 0;
            $display("  Sent beat %0d: %h", i, burst_data[i]);
        end
        
        $display("  Waiting for burst write response...");
        
        // Wait for response with timeout
        timeout = 0;
        while (timeout < 300) begin
            @(posedge clk);
            if (axi_bvalid) begin
                response_received = 1;
                timeout = 300;
            end
            timeout = timeout + 1;
        end
        
        if (response_received) begin
            $display("  Burst write response received at time %t", $time);
            $display("  Write response: %b", axi_bresp);
            if (axi_bresp != 2'b00) begin
                $display("  ERROR: Write response not OKAY (got %b)", axi_bresp);
                test_passed = 0;
            end
        end else begin
            $display("  ERROR: Burst write response timeout!");
            test_passed = 0;
        end
        
        // Output result
        if (test_passed) begin
            $display("  ✓ TEST %0d: PASS", test_counter);
        end else begin
            $display("  ✗ TEST %0d: FAIL", test_counter);
            all_tests_passed = 0;
        end
        
        #(CLK_PERIOD*50);
    end
endtask

// ====================================================
// Test 5: Burst Read of 4 Beats
// ====================================================
task test_burst_read;
    reg [31:0] timeout;
    reg [63:0] expected_data [0:3];
    reg [3:0] beats_received;
    begin
        test_counter = test_counter + 1;
        test_passed = 1;
        beats_received = 0;
        
        $display("========================================");
        $display("Test %0d: Burst Read (4 beats)", test_counter);
        $display("========================================");
        
        #(CLK_PERIOD*50);
        
        axi_rready = 1'b1;
        
        // Expected data from previous burst write
        expected_data[0] = 64'h1111111122222222;
        expected_data[1] = 64'h3333333344444444;
        expected_data[2] = 64'h5555555566666666;
        expected_data[3] = 64'h7777777788888888;
        
        // Start burst read
        @(posedge clk);
        axi_araddr = 32'h1000_2000;
        axi_arlen = 3;  // 4 beats
        axi_arvalid = 1;
        
        @(posedge clk);
        axi_arvalid = 0;
        
        $display("  Waiting for burst read data...");
        
        timeout = 0;
        while (beats_received < 4 && timeout < 400) begin
            @(posedge clk);
            if (axi_rvalid) begin
                $display("  Read beat %0d: %h at time %t", beats_received, axi_rdata, $time);
                
                // Verify data
                if (axi_rdata !== expected_data[beats_received]) begin
                    $display("    ERROR: Beat %0d mismatch", beats_received);
                    $display("      Expected: %h", expected_data[beats_received]);
                    $display("      Got:      %h", axi_rdata);
                    test_passed = 0;
                end
                
                beats_received = beats_received + 1;
                timeout = 0;
            end
            timeout = timeout + 1;
        end
        
        if (beats_received == 4) begin
            $display("  All 4 read beats received successfully");
        end else begin
            $display("  ERROR: Only %0d of 4 read beats received", beats_received);
            test_passed = 0;
        end
        
        // Output result
        if (test_passed) begin
            $display("  ✓ TEST %0d: PASS", test_counter);
        end else begin
            $display("  ✗ TEST %0d: FAIL", test_counter);
            all_tests_passed = 0;
        end
        
        #(CLK_PERIOD*50);
    end
endtask

// ====================================================
// Test 6: Unaligned Access
// ====================================================
task test_unaligned_access;
    reg [31:0] timeout;
    begin
        test_counter = test_counter + 1;
        test_passed = 1;
        response_received = 0;
        
        $display("========================================");
        $display("Test %0d: Unaligned Access (offset 4)", test_counter);
        $display("========================================");
        
        #(CLK_PERIOD*50);
        
        axi_bready = 1'b1;
        
        // Write to unaligned address (4-byte offset)
        @(posedge clk);
        axi_awaddr = 32'h1000_3004;  // 4-byte offset
        axi_awlen = 0;
        axi_awvalid = 1;
        
        @(posedge clk);
        axi_awvalid = 0;
        
        while (!axi_awready) @(posedge clk);
        @(posedge clk);
        
        @(posedge clk);
        axi_wdata = 64'h1122334455667788;
        axi_wstrb = 8'hF0;  // Only upper 4 bytes
        axi_wlast = 1;
        axi_wvalid = 1;
        
        @(posedge clk);
        axi_wvalid = 0;
        
        $display("  Waiting for unaligned write response...");
        
        // Wait for response with timeout
        timeout = 0;
        while (timeout < 200) begin
            @(posedge clk);
            if (axi_bvalid) begin
                response_received = 1;
                timeout = 200;
            end
            timeout = timeout + 1;
        end
        
        if (response_received) begin
            $display("  Unaligned write completed at time %t", $time);
            if (axi_bresp != 2'b00) begin
                $display("  ERROR: Write response not OKAY");
                test_passed = 0;
            end
        end else begin
            $display("  ERROR: Unaligned write timeout!");
            test_passed = 0;
        end
        
        if (test_passed) begin
            $display("  ✓ TEST %0d: PASS", test_counter);
        end else begin
            $display("  ✗ TEST %0d: FAIL", test_counter);
            all_tests_passed = 0;
        end
        
        #(CLK_PERIOD*50);
    end
endtask

// ====================================================
// Test 7: Multiple Rows Access
// ====================================================
task test_multiple_rows;
    reg [31:0] timeout;
    reg [31:0] base_addr;
    reg [63:0] write_data;
    reg [31:0] row;
    begin
        test_counter = test_counter + 1;
        test_passed = 1;
        response_received = 0;
        
        $display("========================================");
        $display("Test %0d: Multiple Rows Access", test_counter);
        $display("========================================");
        
        axi_bready = 1'b1;
        
        // Write to 4 different rows
        for (i = 0; i < 4; i = i + 1) begin
            base_addr = 32'h1000_4000 + (i * 32'h800);  // Each row is 1024 bytes apart
            write_data = {16'hA000 + i, 16'hB000 + i, 16'hC000 + i, 16'hD000 + i};
            row = get_row(base_addr);
            
            $display("  Writing to Row %0d, Addr: %h, Data: %h", row, base_addr, write_data);
            
            @(posedge clk);
            axi_awaddr = base_addr;
            axi_awlen = 0;
            axi_awvalid = 1;
            
            @(posedge clk);
            axi_awvalid = 0;
            
            while (!axi_awready) @(posedge clk);
            @(posedge clk);
            
            @(posedge clk);
            axi_wdata = write_data;
            axi_wstrb = 8'hFF;
            axi_wlast = 1;
            axi_wvalid = 1;
            
            @(posedge clk);
            axi_wvalid = 0;
            
            // Wait for write response
            timeout = 0;
            response_received = 0;
            while (timeout < 200) begin
                @(posedge clk);
                if (axi_bvalid) begin
                    response_received = 1;
                    timeout = 200;
                end
                timeout = timeout + 1;
            end
            
            if (!response_received) begin
                $display("  ERROR: Write timeout for row %0d", row);
                test_passed = 0;
            end
            #(CLK_PERIOD*20);
        end
        
        if (test_passed) begin
            $display("  ✓ TEST %0d: PASS", test_counter);
        end else begin
            $display("  ✗ TEST %0d: FAIL", test_counter);
            all_tests_passed = 0;
        end
        
        #(CLK_PERIOD*50);
    end
endtask

// ====================================================
// Test 8: Different Bank Access
// ====================================================
task test_different_bank;
    reg [31:0] timeout;
    reg [31:0] base_addr;
    reg [63:0] write_data;
    reg [31:0] bank;
    begin
        test_counter = test_counter + 1;
        test_passed = 1;
        
        $display("========================================");
        $display("Test %0d: Different Bank Access", test_counter);
        $display("========================================");
        
        axi_bready = 1'b1;
        axi_rready = 1'b1;
        
        // Test Bank 0
        base_addr = 32'h0000_0000;  // Bank 0
        write_data = 64'h1111111111111111;
        bank = get_bank(base_addr);
        $display("  Testing Bank %0d, Addr: %h", bank, base_addr);
        
        @(posedge clk);
        axi_awaddr = base_addr;
        axi_awlen = 0;
        axi_awvalid = 1;
        
        @(posedge clk);
        axi_awvalid = 0;
        
        while (!axi_awready) @(posedge clk);
        @(posedge clk);
        
        @(posedge clk);
        axi_wdata = write_data;
        axi_wstrb = 8'hFF;
        axi_wlast = 1;
        axi_wvalid = 1;
        
        @(posedge clk);
        axi_wvalid = 0;
        
        timeout = 0;
        while (timeout < 200) begin
            @(posedge clk);
            if (axi_bvalid) timeout = 200;
            timeout = timeout + 1;
        end
        
        #(CLK_PERIOD*50);
        
        // Test Bank 1
        base_addr = 32'h0200_0000;  // Bank 1 (using A[23] and A[9])
        write_data = 64'h2222222222222222;
        bank = get_bank(base_addr);
        $display("  Testing Bank %0d, Addr: %h", bank, base_addr);
        
        @(posedge clk);
        axi_awaddr = base_addr;
        axi_awlen = 0;
        axi_awvalid = 1;
        
        @(posedge clk);
        axi_awvalid = 0;
        
        while (!axi_awready) @(posedge clk);
        @(posedge clk);
        
        @(posedge clk);
        axi_wdata = write_data;
        axi_wstrb = 8'hFF;
        axi_wlast = 1;
        axi_wvalid = 1;
        
        @(posedge clk);
        axi_wvalid = 0;
        
        timeout = 0;
        while (timeout < 200) begin
            @(posedge clk);
            if (axi_bvalid) timeout = 200;
            timeout = timeout + 1;
        end
        
        if (test_passed) begin
            $display("  ✓ TEST %0d: PASS", test_counter);
        end else begin
            $display("  ✗ TEST %0d: FAIL", test_counter);
            all_tests_passed = 0;
        end
        
        #(CLK_PERIOD*50);
    end
endtask

// ====================================================
// SDRAM Command Monitor
// ====================================================
always @(posedge clk) begin
    // Monitor SDRAM commands
    if (!sdram_cs_n) begin
        if (!sdram_ras_n && !sdram_cas_n && sdram_we_n) begin
            $display("  [SDRAM] ACTIVE: Bank=%0d Row=%0d at time %t", 
                     sdram_ba, sdram_addr, $time);
        end
        else if (sdram_ras_n && !sdram_cas_n && sdram_we_n) begin
            $display("  [SDRAM] READ: Bank=%0d Col=%0d at time %t", 
                     sdram_ba, sdram_addr[8:0], $time);
        end
        else if (sdram_ras_n && !sdram_cas_n && !sdram_we_n) begin
            $display("  [SDRAM] WRITE: Bank=%0d Col=%0d at time %t", 
                     sdram_ba, sdram_addr[8:0], $time);
        end
        else if (!sdram_ras_n && sdram_cas_n && sdram_we_n) begin
            $display("  [SDRAM] PRECHARGE at time %t", $time);
        end
        else if (sdram_ras_n && sdram_cas_n && !sdram_we_n) begin
            $display("  [SDRAM] MODE REGISTER SET at time %t", $time);
        end
        else if (!sdram_ras_n && !sdram_cas_n && !sdram_we_n) begin
            $display("  [SDRAM] REFRESH at time %t", $time);
        end
    end
end

// ====================================================
// AXI Monitor
// ====================================================
always @(posedge clk) begin
    if (axi_awvalid && axi_awready) begin
        $display("  [AXI] Write Address: addr=%h len=%0d at time %t",
                 axi_awaddr, axi_awlen, $time);
    end
    if (axi_wvalid && axi_wready) begin
        $display("  [AXI] Write Data: data=%h last=%b at time %t",
                 axi_wdata, axi_wlast, $time);
    end
    if (axi_bvalid && axi_bready) begin
        $display("  [AXI] Write Response: resp=%b at time %t", axi_bresp, $time);
    end
    if (axi_arvalid && axi_arready) begin
        $display("  [AXI] Read Address: addr=%h len=%0d at time %t",
                 axi_araddr, axi_arlen, $time);
    end
    if (axi_rvalid && axi_rready) begin
        $display("  [AXI] Read Data: data=%h last=%b at time %t",
                 axi_rdata, axi_rlast, $time);
    end
end

// ====================================================
// Avalon Monitor
// ====================================================
always @(posedge clk) begin
    if (avm_write) begin
        $display("  [AVALON] Bridge Write: addr=%h data=%h be=%b", 
                 avm_address, avm_writedata, avm_byteenable);
    end
    if (avm_read) begin
        $display("  [AVALON] Bridge Read: addr=%h", avm_address);
    end
    if (avs_readdatavalid) begin
        $display("  [AVALON] Read Data: %h", avs_readdata);
    end
end

endmodule
