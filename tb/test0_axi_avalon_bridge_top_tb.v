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

// ====================================================
// Testbench Signals
// ====================================================
reg                                 clk;
reg                                 reset_n;

// AXI Master Simulator Interface
reg  [AXI_ID_WIDTH-1:0]             s_awid;
reg  [AXI_ADDR_WIDTH-1:0]           s_awaddr;
reg  [7:0]                          s_awlen;
reg  [2:0]                          s_awsize;
reg  [1:0]                          s_awburst;
reg                                 s_awvalid;
wire                                s_awready;

reg  [AXI_DATA_WIDTH-1:0]           s_wdata;
reg  [AXI_DATA_WIDTH/8-1:0]         s_wstrb;
reg                                 s_wlast;
reg                                 s_wvalid;
wire                                s_wready;

wire [AXI_ID_WIDTH-1:0]             s_bid;
wire [1:0]                          s_bresp;
wire                                s_bvalid;
reg                                 s_bready;

reg  [AXI_ID_WIDTH-1:0]             s_arid;
reg  [AXI_ADDR_WIDTH-1:0]           s_araddr;
reg  [7:0]                          s_arlen;
reg  [2:0]                          s_arsize;
reg  [1:0]                          s_arburst;
reg                                 s_arvalid;
wire                                s_arready;

wire [AXI_ID_WIDTH-1:0]             s_rid;
wire [AXI_DATA_WIDTH-1:0]           s_rdata;
wire [1:0]                          s_rresp;
wire                                s_rlast;
wire                                s_rvalid;
reg                                 s_rready;

// Avalon Slave Simulator Interface
wire [AVALON_ADDR_WIDTH-1:0]        m_address;
wire                                m_write;
wire                                m_read;
wire [AVALON_DATA_WIDTH-1:0]        m_writedata;
reg  [AVALON_DATA_WIDTH-1:0]        m_readdata;
wire [1:0]                          m_byteenable;
reg                                 m_waitrequest;
reg                                 m_readdatavalid;
wire                                m_burstcount;

// Test control
integer i, j, k;
reg [63:0] expected;
reg [7:0] read_beat;
reg [7:0] read_sub;
reg [31:0] test_counter;
reg        test_passed;
reg        all_tests_passed;

// Timeout counter
reg [31:0] timeout_counter;
reg        response_received;

// ====================================================
// Instantiate DUT with parameters
// ====================================================
axi_burst_master_to_avalon16 #(
    .AXI_ADDR_WIDTH(AXI_ADDR_WIDTH),
    .AXI_DATA_WIDTH(AXI_DATA_WIDTH),
    .AVALON_ADDR_WIDTH(AVALON_ADDR_WIDTH),
    .AVALON_DATA_WIDTH(AVALON_DATA_WIDTH),
    .AXI_MAX_BURST(16),
    .FIFO_DEPTH(32)
) u_dut (
    .clk            (clk),
    .reset_n        (reset_n),
    
    // AXI Slave Interface
    .s_awid         (s_awid),
    .s_awaddr       (s_awaddr),
    .s_awlen        (s_awlen),
    .s_awsize       (s_awsize),
    .s_awburst      (s_awburst),
    .s_awvalid      (s_awvalid),
    .s_awready      (s_awready),
    
    .s_wdata        (s_wdata),
    .s_wstrb        (s_wstrb),
    .s_wlast        (s_wlast),
    .s_wvalid       (s_wvalid),
    .s_wready       (s_wready),
    
    .s_bid          (s_bid),
    .s_bresp        (s_bresp),
    .s_bvalid       (s_bvalid),
    .s_bready       (s_bready),
    
    .s_arid         (s_arid),
    .s_araddr       (s_araddr),
    .s_arlen        (s_arlen),
    .s_arsize       (s_arsize),
    .s_arburst      (s_arburst),
    .s_arvalid      (s_arvalid),
    .s_arready      (s_arready),
    
    .s_rid          (s_rid),
    .s_rdata        (s_rdata),
    .s_rresp        (s_rresp),
    .s_rlast        (s_rlast),
    .s_rvalid       (s_rvalid),
    .s_rready       (s_rready),
    
    // Avalon Master Interface
    .m_address      (m_address),
    .m_write        (m_write),
    .m_read         (m_read),
    .m_writedata    (m_writedata),
    .m_readdata     (m_readdata),
    .m_byteenable   (m_byteenable),
    .m_waitrequest  (m_waitrequest),
    .m_readdatavalid(m_readdatavalid),
    .m_burstcount   (m_burstcount)
);

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
    // Initialize signals
    reset_n = 0;
    s_awaddr = 0;
    s_awlen = 0;
    s_awsize = 3'b011;  // 8 bytes
    s_awburst = 2'b01;  // INCR
    s_awvalid = 0;
    s_wdata = 0;
    s_wstrb = 8'hFF;
    s_wlast = 0;
    s_wvalid = 0;
    s_bready = 1;
    s_araddr = 0;
    s_arlen = 0;
    s_arsize = 3'b011;
    s_arburst = 2'b01;
    s_arvalid = 0;
    s_rready = 1;
    
    m_readdata = 0;
    m_waitrequest = 0;
    m_readdatavalid = 0;
    
    all_tests_passed = 1;
    test_counter = 0;
    
    $display("========================================");
    $display("AXI to Avalon Bridge Testbench");
    $display("========================================");
    $display("Applying reset...");
    $display("========================================");
    #(CLK_PERIOD*20);
    reset_n = 1;
    #(CLK_PERIOD*20);
    
    // Run tests
    test_single_write();
    test_burst_write();
    test_single_read();
    test_burst_read();
    test_write_with_waitrequest();
    test_read_with_waitrequest();
    test_unaligned_access();
    
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
        $display("Test %0d: Single 64-bit write", test_counter);
        $display("========================================");
        
        s_bready = 1'b1;
        
        @(posedge clk);
        s_awaddr = 32'h1000_0000;
        s_awlen = 0;
        s_awvalid = 1;
        
        @(posedge clk);
        s_awvalid = 0;
        
        // Wait for awready
        while (!s_awready) @(posedge clk);
        @(posedge clk);
        
        @(posedge clk);
        s_wdata = 64'h0123456789ABCDEF;
        s_wstrb = 8'hFF;
        s_wlast = 1;
        s_wvalid = 1;
        
        @(posedge clk);
        s_wvalid = 0;
        s_wlast = 0;
        
        $display("  Waiting for write response...");
        
        // Wait for response with timeout
        timeout = 0;
        while (timeout < 200) begin
            @(posedge clk);
            if (s_bvalid) begin
                response_received = 1;
                timeout = 200; // exit loop
            end
            timeout = timeout + 1;
        end
        
        if (response_received) begin
            $display("  Write response received at time %t", $time);
            $display("  Write response: %b", s_bresp);
            if (s_bresp != 2'b00) begin
                $display("  ERROR: Write response not OKAY (got %b)", s_bresp);
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
        
        #(CLK_PERIOD*20);
    end
endtask

// ====================================================
// Test 2: Burst Write of 4 Beats
// ====================================================
task test_burst_write;
    reg [31:0] timeout;
    begin
        test_counter = test_counter + 1;
        test_passed = 1;
        response_received = 0;
        
        $display("========================================");
        $display("Test %0d: Burst write of 4 beats", test_counter);
        $display("========================================");
        
        #(CLK_PERIOD*20);
        
        s_bready = 1'b1;
        
        @(posedge clk);
        s_awaddr = 32'h1000_0100;
        s_awlen = 3;  // 4 beats
        s_awvalid = 1;
        
        @(posedge clk);
        s_awvalid = 0;
        
        // Wait for awready
        while (!s_awready) @(posedge clk);
        @(posedge clk);
        
        for (i = 0; i < 4; i = i + 1) begin
            @(posedge clk);
            s_wdata = {56'b0, i[7:0]};
            s_wstrb = 8'hFF;
            s_wlast = (i == 3);
            s_wvalid = 1;
            
            @(posedge clk);
            s_wvalid = 0;
        end
        
        $display("  Waiting for burst write response...");
        
        // Wait for response with timeout
        timeout = 0;
        while (timeout < 300) begin
            @(posedge clk);
            if (s_bvalid) begin
                response_received = 1;
                timeout = 300; // exit loop
            end
            timeout = timeout + 1;
        end
        
        if (response_received) begin
            $display("  Burst write response received at time %t", $time);
            $display("  Write response: %b", s_bresp);
            if (s_bresp != 2'b00) begin
                $display("  ERROR: Write response not OKAY (got %b)", s_bresp);
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
        
        #(CLK_PERIOD*20);
    end
endtask

// ====================================================
// Test 3: Single 64-bit Read
// ====================================================
task test_single_read;
    reg [31:0] timeout;
    begin
        test_counter = test_counter + 1;
        test_passed = 1;
        
        $display("========================================");
        $display("Test %0d: Single 64-bit read", test_counter);
        $display("========================================");
        
        #(CLK_PERIOD*20);
        
        s_rready = 1'b1;
        
        // Start generating Avalon responses in a separate always block
        fork: single_read_fork
            begin: avalon_resp_gen
                #(CLK_PERIOD*10);
                for (i = 0; i < 4; i = i + 1) begin
                    @(posedge clk);
                    m_readdata = 16'hA000 + i;
                    m_readdatavalid = 1;
                    $display("  [AVALON RESP] Sending read data: %h at time %t", 
                             m_readdata, $time);
                    @(posedge clk);
                    m_readdatavalid = 0;
                end
            end
            
            begin: axi_read_start
                @(posedge clk);
                s_araddr = 32'h2000_0000;
                s_arlen = 0;
                s_arvalid = 1;
                
                @(posedge clk);
                s_arvalid = 0;
                
                // Wait for read data with timeout
                timeout = 0;
                while (timeout < 200) begin
                    @(posedge clk);
                    if (s_rvalid) begin
                        timeout = 200; // exit loop
                    end
                    timeout = timeout + 1;
                end
            end
        join
        
        disable single_read_fork;
        
        // Check if we got valid data
        if (s_rvalid) begin
            $display("  Read data received: %h at time %t", s_rdata, $time);
            
            // Verify data
            expected = {16'hA003, 16'hA002, 16'hA001, 16'hA000};
            if (s_rdata !== expected) begin
                $display("  ERROR: Read data mismatch.");
                $display("    Expected: %h", expected);
                $display("    Got:      %h", s_rdata);
                test_passed = 0;
            end else begin
                $display("    Data correct: %h", s_rdata);
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
        
        #(CLK_PERIOD*20);
    end
endtask

// ====================================================
// Test 4: Burst Read of 4 Beats
// ====================================================
task test_burst_read;
    reg [31:0] timeout;
    reg [3:0] beats_received;
    begin
        test_counter = test_counter + 1;
        test_passed = 1;
        beats_received = 0;
        
        $display("========================================");
        $display("Test %0d: Burst read of 4 beats", test_counter);
        $display("========================================");
        
        #(CLK_PERIOD*20);
        
        s_rready = 1'b1;
        
        fork: burst_read_fork
            begin: avalon_burst_resp
                #(CLK_PERIOD*10);
                for (i = 0; i < 4; i = i + 1) begin
                    for (j = 0; j < 4; j = j + 1) begin
                        @(posedge clk);
                        m_readdata = {4'hB, i[3:0], j[3:0]};
                        m_readdatavalid = 1;
                        $display("  [AVALON RESP] Beat %0d, Sub %0d: data=%h", 
                                 i, j, m_readdata);
                        @(posedge clk);
                        m_readdatavalid = 0;
                    end
                end
            end
            
            begin: axi_burst_read
                @(posedge clk);
                s_araddr = 32'h2000_0100;
                s_arlen = 3;  // 4 beats
                s_arvalid = 1;
                
                @(posedge clk);
                s_arvalid = 0;
                
                timeout = 0;
                while (beats_received < 4 && timeout < 400) begin
                    @(posedge clk);
                    if (s_rvalid) begin
                        $display("  Read beat %0d: %h at time %t", beats_received, s_rdata, $time);
                        beats_received = beats_received + 1;
                        timeout = 0;
                    end
                    timeout = timeout + 1;
                end
            end
        join
        
        disable burst_read_fork;
        
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
        
        #(CLK_PERIOD*20);
    end
endtask

// ====================================================
// Test 5: Write with Waitrequest
// ====================================================
task test_write_with_waitrequest;
    reg [31:0] timeout;
    begin
        test_counter = test_counter + 1;
        test_passed = 1;
        response_received = 0;
        
        $display("========================================");
        $display("Test %0d: Write with waitrequest", test_counter);
        $display("========================================");
        
        #(CLK_PERIOD*20);
        
        s_bready = 1'b1;
        m_waitrequest = 1;
        
        @(posedge clk);
        s_awaddr = 32'h3000_0000;
        s_awlen = 0;
        s_awvalid = 1;
        
        @(posedge clk);
        s_awvalid = 0;
        
        while (!s_awready) @(posedge clk);
        @(posedge clk);
        
        @(posedge clk);
        s_wdata = 64'hDEADBEEFCAFEBABE;
        s_wlast = 1;
        s_wvalid = 1;
        
        $display("  Waitrequest active, holding write...");
        #(CLK_PERIOD*10);
        $display("  Releasing waitrequest...");
        m_waitrequest = 0;
        
        @(posedge clk);
        s_wvalid = 0;
        
        $display("  Waiting for write response...");
        
        // Wait for response with timeout
        timeout = 0;
        while (timeout < 200) begin
            @(posedge clk);
            if (s_bvalid) begin
                response_received = 1;
                timeout = 200; // exit loop
            end
            timeout = timeout + 1;
        end
        
        if (response_received) begin
            $display("  Write with waitrequest completed at time %t", $time);
            if (s_bresp != 2'b00) begin
                $display("  ERROR: Write response not OKAY");
                test_passed = 0;
            end
        end else begin
            $display("  ERROR: Write response timeout!");
            test_passed = 0;
        end
        
        if (test_passed) begin
            $display("  ✓ TEST %0d: PASS", test_counter);
        end else begin
            $display("  ✗ TEST %0d: FAIL", test_counter);
            all_tests_passed = 0;
        end
        
        #(CLK_PERIOD*20);
    end
endtask

// ====================================================
// Test 6: Read with Waitrequest
// ====================================================
task test_read_with_waitrequest;
    reg [31:0] timeout;
    begin
        test_counter = test_counter + 1;
        test_passed = 1;
        
        $display("========================================");
        $display("Test %0d: Read with waitrequest", test_counter);
        $display("========================================");
        
        #(CLK_PERIOD*20);
        
        s_rready = 1'b1;
        m_waitrequest = 1;
        
        fork: read_wait_fork
            begin: delayed_avalon
                #(CLK_PERIOD*20);
                $display("  Releasing waitrequest...");
                m_waitrequest = 0;
                #(CLK_PERIOD*2);
                for (i = 0; i < 4; i = i + 1) begin
                    @(posedge clk);
                    m_readdata = 16'hC000 + i;
                    m_readdatavalid = 1;
                    @(posedge clk);
                    m_readdatavalid = 0;
                end
            end
            
            begin: axi_read_wait
                @(posedge clk);
                s_araddr = 32'h4000_0000;
                s_arlen = 0;
                s_arvalid = 1;
                
                @(posedge clk);
                s_arvalid = 0;
                
                timeout = 0;
                while (timeout < 300) begin
                    @(posedge clk);
                    if (s_rvalid) begin
                        timeout = 300; // exit loop
                    end
                    timeout = timeout + 1;
                end
            end
        join
        
        disable read_wait_fork;
        
        if (s_rvalid) begin
            $display("  Read with waitrequest completed: %h at time %t", s_rdata, $time);
            expected = {16'hC003, 16'hC002, 16'hC001, 16'hC000};
            if (s_rdata !== expected) begin
                $display("  ERROR: Read data mismatch");
                $display("    Expected: %h", expected);
                $display("    Got:      %h", s_rdata);
                test_passed = 0;
            end
        end else begin
            $display("  ERROR: No read data received");
            test_passed = 0;
        end
        
        if (test_passed) begin
            $display("  ✓ TEST %0d: PASS", test_counter);
        end else begin
            $display("  ✗ TEST %0d: FAIL", test_counter);
            all_tests_passed = 0;
        end
        
        #(CLK_PERIOD*20);
    end
endtask

// ====================================================
// Test 7: Unaligned Access
// ====================================================
task test_unaligned_access;
    reg [31:0] timeout;
    begin
        test_counter = test_counter + 1;
        test_passed = 1;
        response_received = 0;
        
        $display("========================================");
        $display("Test %0d: Unaligned access", test_counter);
        $display("========================================");
        
        #(CLK_PERIOD*20);
        
        s_bready = 1'b1;
        
        @(posedge clk);
        s_awaddr = 32'h5000_0004;  // 4-byte offset
        s_awlen = 0;
        s_awvalid = 1;
        
        @(posedge clk);
        s_awvalid = 0;
        
        while (!s_awready) @(posedge clk);
        @(posedge clk);
        
        @(posedge clk);
        s_wdata = 64'h1122334455667788;
        s_wstrb = 8'hF0;  // Only upper 4 bytes
        s_wlast = 1;
        s_wvalid = 1;
        
        @(posedge clk);
        s_wvalid = 0;
        
        $display("  Waiting for unaligned write response...");
        
        // Wait for response with timeout
        timeout = 0;
        while (timeout < 200) begin
            @(posedge clk);
            if (s_bvalid) begin
                response_received = 1;
                timeout = 200; // exit loop
            end
            timeout = timeout + 1;
        end
        
        if (response_received) begin
            $display("  Unaligned write completed at time %t", $time);
            if (s_bresp != 2'b00) begin
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
        
        #(CLK_PERIOD*20);
    end
endtask

// ====================================================
// Avalon Monitor
// ====================================================
always @(posedge clk) begin
    if (m_write && !m_waitrequest) begin
        $display("  [AVALON] Write: addr=%h data=%h be=%b at time %t", 
                 m_address, m_writedata, m_byteenable, $time);
    end
    if (m_read && !m_waitrequest) begin
        $display("  [AVALON] Read : addr=%h at time %t", m_address, $time);
    end
    if (m_readdatavalid) begin
        $display("  [AVALON] Data : addr=%h data=%h at time %t", 
                 m_address, m_readdata, $time);
    end
end

// ====================================================
// AXI Monitor
// ====================================================
always @(posedge clk) begin
    if (s_awvalid && s_awready) begin
        $display("  [AXI] Write Address: addr=%h len=%0d at time %t",
                 s_awaddr, s_awlen, $time);
    end
    if (s_wvalid && s_wready) begin
        $display("  [AXI] Write Data: data=%h last=%b at time %t",
                 s_wdata, s_wlast, $time);
    end
    if (s_bvalid && s_bready) begin
        $display("  [AXI] Write Response: resp=%b at time %t", s_bresp, $time);
    end
    if (s_arvalid && s_arready) begin
        $display("  [AXI] Read Address: addr=%h len=%0d at time %t",
                 s_araddr, s_arlen, $time);
    end
    if (s_rvalid && s_rready) begin
        $display("  [AXI] Read Data: data=%h last=%b at time %t",
                 s_rdata, s_rlast, $time);
    end
end

endmodule
