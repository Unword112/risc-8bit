module alu(
    input [7:0] A,  // rt
    input [7:0] B,  // rd หรือ immediate
    input [1:0] opcode, 
    output reg [7:0] result
);
    always @(*) begin
        case (opcode)
            2'b00:  result = A + B;    // ADD (rs = rt + rd)
            2'b01:  result = A + B;    // ADDI (rs = rt + imm)
            2'b10:  result = (A == B) ? 8'b1 : 8'b0; // BEQ
            default:  result = 8'b0;
        endcase
    end
endmodule

module register(
    input wire clk,  
    input wire reset,  
    input wire we,  
    input wire [1:0] rs,   
    input wire [1:0] rt,  
    input wire [1:0] rd,  
    input wire [7:0] data_in,  
    output wire [7:0] rt_out,  
    output wire [7:0] rd_out,
    output [7:0] r0,
    output [7:0] r1,
    output [7:0] r2,
    output [7:0] r3
);
    reg [7:0] r [3:0];  // 4 general-purpose registers
    assign r0 = r[0];
    assign r1 = r[1];
    assign r2 = r[2];
    assign r3 = r[3];

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            r[0] <= 8'b0;
            r[1] <= 8'b0;
            r[2] <= 8'b0;
            r[3] <= 8'b0;
        end else if (we) begin
            r[rs] <= data_in;  
        end
    end

    assign rt_out = r[rt];
    assign rd_out = r[rd];
endmodule

module controlUnit(
    input [7:0] instruction, 
    output reg [1:0] opcode, 
    output reg [1:0] rs,    //  rs ให้เป็นตัวปลายทาง
    output reg [1:0] rt, 
    output reg [1:0] rd, 
    output reg [3:0] imm_addr, 
    output reg reg_write, 
    output reg alu_src, 
    output reg jump, 
    output reg beq
);
    always @(*) begin
        opcode = instruction[7:6];
        jump = 0;
        beq = 0;

        case (opcode)
            2'b00: begin // ADD
                rs = instruction[5:4]; //ให้ rs เป็นตัวเก็บผลลัพธ์
                rt = instruction[3:2]; 
                rd = instruction[1:0]; 
                reg_write = 1;
                alu_src = 0;
                imm_addr = 4'b0;
            end
            2'b01: begin // ADDI
                rs = instruction[5:4]; // rs เก็บผลลัพธ์
                rt = instruction[3:2];
                imm_addr = instruction[1:0];
                reg_write = 1;
                alu_src = 1;
                rd = 0;
            end
            2'b10: begin // BEQ
                rs = instruction[5:4];
                rt = instruction[3];
                imm_addr = instruction[2:0];
                reg_write = 0;
                alu_src = 0;
                beq = 1;
            end
            2'b11: begin // JUMP
                imm_addr = instruction[3:0];
                jump = 1;
                reg_write = 0;
                alu_src = 0;
                rs = 0;
                rt = 0;
                rd = 0;
            end
            default: begin
                reg_write = 0;
                alu_src = 0;
                jump = 0;
                beq = 0;
                rs = 0;
                rt = 0;
                rd = 0;
                imm_addr = 4'b0;
            end
        endcase
    end
endmodule

module pc_module(
    input wire clk,
    input wire reset,
    input wire jump,
    input wire beq,
    input wire [3:0] imm_addr,
    input wire branch_taken,
    output reg [7:0] pc
);
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            pc <= 0;
        end else begin
            if (jump) begin
                $display("Jumping to PC=%d", {4'b0, imm_addr});
                pc <= {4'b0, imm_addr};
            end else if (branch_taken) begin
                $display("Branching to PC=%d", pc + {{4{imm_addr[3]}}, imm_addr});
                pc <= pc + {{4{imm_addr[3]}}, imm_addr};
            end else begin
                pc <= pc + 1;
            end
        end
    end
endmodule

module cpu(
    input wire clk,       
    input wire reset,       
    input wire [7:0] instruction,
    output wire [7:0] result,
    output [7:0] r0,
    output [7:0] r1,
    output [7:0] r2,
    output [7:0] r3
);
    wire [7:0] pc;  
    wire [7:0] alu_result, reg_rt_out, reg_rd_out;
    wire [1:0] opcode, rs, rt, rd; 
    wire reg_write, alu_src, jump, beq;
    wire [3:0] imm_addr;  
    wire [7:0] alu_b_input;
    wire branch_taken;

    controlUnit cu (
        .instruction(instruction),   
        .opcode(opcode),   
        .rs(rs),   
        .rt(rt),   
        .rd(rd),   
        .imm_addr(imm_addr),   
        .reg_write(reg_write),   
        .alu_src(alu_src),   
        .jump(jump),   
        .beq(beq)
    );

    register reg_file (
        .clk(clk),   
        .reset(reset),   
        .we(reg_write),   
        .rs(rs),   
        .rt(rt),   
        .rd(rd),   
        .data_in(alu_result),   
        .rt_out(reg_rt_out),   
        .rd_out(reg_rd_out),
        .r0(r0),
        .r1(r1),
        .r2(r2),
        .r3(r3)
    );

    assign alu_b_input = alu_src ? {{4{imm_addr[3]}}, imm_addr} : reg_rd_out;

    alu alu_inst (
        .A(reg_rt_out),   
        .B(alu_b_input),   
        .opcode(opcode),   
        .result(alu_result)
    );

    assign result = alu_result;
    
    assign branch_taken = beq && (alu_result == 8'b1);
    
    pc_module pc_inst (
        .clk(clk),
        .reset(reset),
        .jump(jump),
        .beq(beq),
        .imm_addr(imm_addr),
        .branch_taken(branch_taken),
        .pc(pc)
    );
    
endmodule

module cpu_tb;
    reg clk, reset;
    reg [7:0] instruction;
    wire [7:0] result;
    wire [7:0] r0;
	wire [7:0] r1;
	wire [7:0] r2;
	wire [7:0] r3;

    cpu DUT (
        .clk(clk),
        .reset(reset),
        .instruction(instruction),
        .result(result),
        .r0(r0),
        .r1(r1),
        .r2(r2),
        .r3(r3) 
    );

    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    initial begin
        $monitor("Time=%0t PC=%d rs=%d rt=%d rd=%d instruction=%b alu_b_input=%d r0=%d r1=%d r2=%d r3=%d", 
         $time, DUT.pc, DUT.rs, DUT.rt, DUT.rd, instruction,  // ใช้ rs, rt, rd จาก DUT โดยตรง
         DUT.alu_b_input, r0, r1, r2, r3);

        reset = 1;
        instruction = 8'b0;
        #10;
        reset = 0;
        
        #350; // Run for some time
        $display("b result: Final b result = %d (expected r2 = 45)", DUT.reg_file.r[2]);
        $finish;
    end
    
    always @(DUT.pc) begin
        case (DUT.pc)
            0: instruction = 8'b01000011; // ADDI R0, R0, 3
            1: instruction = 8'b01000011; // ADDI R0, R0, 3
            2: instruction = 8'b01000011; // ADDI R0, R0, 3
            3: instruction = 8'b01000001; // ADDI R0, R0, 1
            4: instruction = 8'b01010100; // ADDI R1, R1, 0
            5: instruction = 8'b01101000; // ADDI R2, R2, 0
            6: instruction = 8'b10001111; // BEQ R0, R1, +5 END LOOP
            7: instruction = 8'b00101001; // ADD R2, R2, R1
            8: instruction = 8'b01010101; // ADDI R1, R1, 1
            9: instruction = 8'b11000111; // JUMP -3
        endcase
    end
endmodule
