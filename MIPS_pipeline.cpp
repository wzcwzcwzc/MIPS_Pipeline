#include<iostream>
#include<string>
#include<vector>
#include<bitset>
#include<fstream>
using namespace std;
#define MemSize 1000 // memory size, in reality, the memory size should be 2^32, but for this lab, for the space reason, we keep it as this large number, but the memory is still 32-bit addressable.
#define ADDU 1
#define SUBU 3
/**
 * This program is simulating a 5 stage MIPS pipeline in C++ which can handle different types of hazards
 * @author Barry Wang
 * @version 1.0
 * @time 10.31.2019
 * */

/**
 * The MIPS pipeline can be divided into IF, ID, EXE, MEM, WB stages
 * each stage can be described as struct in C++
 * following code is to define the structure of each stage in the pipeline
 * */
struct IFStruct {
    bitset<32>  PC;
    bool        nop;
};

struct IDStruct {
    bitset<32>  Instr;
    bool        nop;
};

//add bool is_beq to keep state of last instruction
struct EXStruct {
    bitset<32>  Read_data1;
    bitset<32>  Read_data2;
    bitset<16>  Imm;
    bitset<5>   Rs;
    bitset<5>   Rt;
    bitset<5>   Wrt_reg_addr;
    bool        is_I_type;
    bool        rd_mem;
    bool        wrt_mem;
    bool        alu_op;     //1 for addu, lw, sw, 0 for subu
    bool        wrt_enable;
    bool        nop;
    bool        is_beq;
};

//add bool component is_I_type_MEM in order to store the state of last EX
struct MEMStruct {
    bitset<32>  ALUresult;
    bitset<32>  Store_data;
    bitset<5>   Rs;
    bitset<5>   Rt;
    bitset<5>   Wrt_reg_addr;
    bool        rd_mem;
    bool        wrt_mem;
    bool        wrt_enable;
    bool        nop;
    bool        MEM_is_I_type;
};

//add bool component is_I_type_WB in order to store the state of last MEM
struct WBStruct {
    bitset<32>  Wrt_data;
    bitset<5>   Rs;
    bitset<5>   Rt;
    bitset<5>   Wrt_reg_addr;
    bool        wrt_enable;
    bool        nop;
    bool        WB_is_I_type;
};

struct stateStruct {
    IFStruct    IF;
    IDStruct    ID;
    EXStruct    EX;
    MEMStruct   MEM;
    WBStruct    WB;
};

/**
 * class RF is used as register in processor
 * it can support both write and read operation based on kinds of instruction fetched
 * during IF stage.
 * */
class RF
{
public:
    bitset<32> Reg_data;

    RF()
    {
        Registers.resize(32);
        Registers[0] = bitset<32> (0);
    }

    bitset<32> readRF(bitset<5> Reg_addr)
    {
        Reg_data = Registers[Reg_addr.to_ulong()];
        return Reg_data;
    }

    void writeRF(bitset<5> Reg_addr, bitset<32> Wrt_reg_data)
    {
        Registers[Reg_addr.to_ulong()] = Wrt_reg_data;
    }

    void outputRF()
    {
        ofstream rfout;
        rfout.open("RFresult.txt",std::ios_base::app);
        if (rfout.is_open())
        {
            rfout<<"State of RF:\t"<<endl;
            for (int j = 0; j<32; j++)
            {
                rfout << Registers[j]<<endl;
            }
        }
        else cout<<"Unable to open file";
        rfout.close();
    }

private:
    vector<bitset<32> >Registers;
};

/**
 * class INSMem is used as instruction memory which can read imem.txt
 * and fetch instructions out of imem file.
 * */
class INSMem
{
public:
    bitset<32> Instruction;
    INSMem()
    {
        IMem.resize(MemSize);
        ifstream imem;
        string line;
        int i=0;
        imem.open("imem.txt");
        if (imem.is_open())
        {
            while (getline(imem,line))
            {
                IMem[i] = bitset<8>(line);
                i++;
            }
        }
        else cout<<"Unable to open file";
        imem.close();
    }

    bitset<32> readInstr(bitset<32> ReadAddress)
    {
        string insmem;
        insmem.append(IMem[ReadAddress.to_ulong()].to_string());
        insmem.append(IMem[ReadAddress.to_ulong()+1].to_string());
        insmem.append(IMem[ReadAddress.to_ulong()+2].to_string());
        insmem.append(IMem[ReadAddress.to_ulong()+3].to_string());
        Instruction = bitset<32>(insmem);		//read instruction memory
        return Instruction;
    }

private:
    vector<bitset<8> > IMem;
};

/**
 * class DataMem is used for MEM stage.
 * it simulates the data memory of processor
 * and it can read data from dmem.txt, write data into dmemresult.txt
 * */
class DataMem
{
public:
    bitset<32> ReadData;
    DataMem()
    {
        DMem.resize(MemSize);
        ifstream dmem;
        string line;
        int i=0;
        dmem.open("dmem.txt");
        if (dmem.is_open())
        {
            while (getline(dmem,line))
            {
                DMem[i] = bitset<8>(line);
                i++;
            }
        }
        else cout<<"Unable to open file";
        dmem.close();
    }

    bitset<32> readDataMem(bitset<32> Address)
    {
        string datamem;
        datamem.append(DMem[Address.to_ulong()].to_string());
        datamem.append(DMem[Address.to_ulong()+1].to_string());
        datamem.append(DMem[Address.to_ulong()+2].to_string());
        datamem.append(DMem[Address.to_ulong()+3].to_string());
        ReadData = bitset<32>(datamem);		//read data memory
        return ReadData;
    }

    void writeDataMem(bitset<32> Address, bitset<32> WriteData)
    {
        DMem[Address.to_ulong()] = bitset<8>(WriteData.to_string().substr(0,8));
        DMem[Address.to_ulong()+1] = bitset<8>(WriteData.to_string().substr(8,8));
        DMem[Address.to_ulong()+2] = bitset<8>(WriteData.to_string().substr(16,8));
        DMem[Address.to_ulong()+3] = bitset<8>(WriteData.to_string().substr(24,8));
    }

    void outputDataMem()
    {
        ofstream dmemout;
        dmemout.open("dmemresult.txt");
        if (dmemout.is_open())
        {
            for (int j = 0; j< 1000; j++)
            {
                dmemout << DMem[j]<<endl;
            }

        }
        else cout<<"Unable to open file";
        dmemout.close();
    }

private:
    vector<bitset<8> > DMem;
};

/**
 * Define what type of instruction each time we fetch from imem.txt
 * and use it to determine R operation or I operation during EXE stage.
 * */
string TypeofInstruction(string &op_code){
    if(op_code == "000000") return "R";
    else if(op_code == "111111") return "halt";
    else if(op_code == "000100") return "beq";
    else if(op_code == "100011") return "lw";
    else if(op_code == "101011") return "sw";
    else return "Invalid Opcode";
}

/**
 * define the function type by decoding the instruction
 * it involves two types of alu operation.
 * you can create more components like or, and operation, etc.
 *
 * @return type of function
 * */
string TypeofFunction(bitset<32> instruction){
    string alu_op = instruction.to_string().substr(29,3);
    if(alu_op == "001") return "addu";
    else if (alu_op == "011") return "subu";
    else return "Invalid AluOP";
}

/**
 * OperandsAddr_R(bitset<32> instruction)
 * it is used for decoding the R-type instruction and get Rs, Rt, Rd
 * */
vector<bitset<5> > OperandsAddr_R(bitset<32> instruction){
    vector<bitset<5> > result(3);
    string rs = instruction.to_string().substr(6,5);
    string rt = instruction.to_string().substr(11,5);
    string rd = instruction.to_string().substr(16,5);
    bitset<5> Rd(rd);
    bitset<5> Rs(rs);
    bitset<5> Rt(rt);

    result[0] = Rs;
    result[1] = Rt;
    result[2] = Rd;

    return result;
}

/**
 * OperandsAddr_I(bitset<32> instruction)
 * it is used for decoding the I-type instruction and get Rs, Rt
 * */
vector<string> OperandsAddr_I(bitset<32> instruction){
    string instr = instruction.to_string();

    string rs = instruction.to_string().substr(6,5);
    string rd = instruction.to_string().substr(11,5);
    string imm = instruction.to_string().substr(16,16);

    vector<string> result(3);
    result[0] = rs;
    result[1] = rd;
    result[2] = imm;

    return result;
}

/**
 * basic ALU components of MIPS processor
 * it supports addu and subu
 * you can modify it by making up other ALU operations
 * */
class ALU
{
public:
    bitset<32> ALUresult;
    bitset<32> ALUOperation (bitset<3> ALUOP, bitset<32> oprand1, bitset<32> oprand2) {
        unsigned long result = 0;

        // addu operation
        if (ALUOP.to_string() == "001") {
            result = oprand1.to_ulong() + oprand2.to_ulong();
        } else if (ALUOP.to_string() == "011") {// subu operation
            result = oprand1.to_ulong() - oprand2.to_ulong();
        }

        bitset<32> res((int) result);
        ALUresult = res;
        return ALUresult;
    }
};

/**
 * it is used for print the state of each flip-flop(cycle)
 * and see differences between each cycle.
 * */
void printState(stateStruct state, int cycle)
{
    ofstream printstate;
    printstate.open("stateresult.txt", std::ios_base::app);
    if (printstate.is_open())
    {
        printstate<<"State after executing cycle:\t"<<cycle<<endl;

        printstate<<"IF.PC:\t"<<state.IF.PC.to_ulong()<<endl;
        printstate<<"IF.nop:\t"<<state.IF.nop<<endl;

        printstate<<"ID.Instr:\t"<<state.ID.Instr<<endl;
        printstate<<"ID.nop:\t"<<state.ID.nop<<endl;

        printstate<<"EX.Read_data1:\t"<<state.EX.Read_data1<<endl;
        printstate<<"EX.Read_data2:\t"<<state.EX.Read_data2<<endl;
        printstate<<"EX.Imm:\t"<<state.EX.Imm<<endl;
        printstate<<"EX.Rs:\t"<<state.EX.Rs<<endl;
        printstate<<"EX.Rt:\t"<<state.EX.Rt<<endl;
        printstate<<"EX.Wrt_reg_addr:\t"<<state.EX.Wrt_reg_addr<<endl;
        printstate<<"EX.is_I_type:\t"<<state.EX.is_I_type<<endl;
        printstate<<"EX.rd_mem:\t"<<state.EX.rd_mem<<endl;
        printstate<<"EX.wrt_mem:\t"<<state.EX.wrt_mem<<endl;
        printstate<<"EX.alu_op:\t"<<state.EX.alu_op<<endl;
        printstate<<"EX.wrt_enable:\t"<<state.EX.wrt_enable<<endl;
        printstate<<"EX.nop:\t"<<state.EX.nop<<endl;

        printstate<<"MEM.ALUresult:\t"<<state.MEM.ALUresult<<endl;
        printstate<<"MEM.Store_data:\t"<<state.MEM.Store_data<<endl;
        printstate<<"MEM.Rs:\t"<<state.MEM.Rs<<endl;
        printstate<<"MEM.Rt:\t"<<state.MEM.Rt<<endl;
        printstate<<"MEM.Wrt_reg_addr:\t"<<state.MEM.Wrt_reg_addr<<endl;
        printstate<<"MEM.rd_mem:\t"<<state.MEM.rd_mem<<endl;
        printstate<<"MEM.wrt_mem:\t"<<state.MEM.wrt_mem<<endl;
        printstate<<"MEM.wrt_enable:\t"<<state.MEM.wrt_enable<<endl;
        printstate<<"MEM.nop:\t"<<state.MEM.nop<<endl;

        printstate<<"WB.Wrt_data:\t"<<state.WB.Wrt_data<<endl;
        printstate<<"WB.Rs:\t"<<state.WB.Rs<<endl;
        printstate<<"WB.Rt:\t"<<state.WB.Rt<<endl;
        printstate<<"WB.Wrt_reg_addr:\t"<<state.WB.Wrt_reg_addr<<endl;
        printstate<<"WB.wrt_enable:\t"<<state.WB.wrt_enable<<endl;
        printstate<<"WB.nop:\t"<<state.WB.nop<<endl;
    }
    else cout<<"Unable to open file";
    printstate.close();
}

/**
 * update pc=pc+4
 * this function is used for calculate new address for program counter
 * */
bitset<32> calProgramCounter(bitset<32> &programCounter){
    programCounter = bitset<32> (programCounter.to_ulong() + 4);
    return programCounter;
}

/**
 * this function is used for sign extend operation on imm in the instruction
 * for calculating data memory addresses
 * */
//signExtend for I-type
bitset<32> signExtend (bitset<16> imm) {
    string finalImmStr;
    if (imm[15] == 0) {
        finalImmStr = "0000000000000000" + imm.to_string();
    } else {
        finalImmStr = "1111111111111111" + imm.to_string();
    }
    return (bitset<32>(finalImmStr));
}

int main()
{
    bitset<32> programCounter = bitset<32>(0);

    RF myRF;
    INSMem myInsMem;
    DataMem myDataMem;
    ALU myALU;

//    IFStruct IF;
//    IDStruct ID;
//    EXStruct EX;
//    MEMStruct MEM;
//    WBStruct WB;

    stateStruct state;
    stateStruct newState;

    vector<bitset<5> > RAddresses;
    vector<string> IAddressess(3);
    //R type
    bitset<5> RAddr_Rs, RAddr_Rt, RAddr_Rd;
    //I type
    bitset<5> IAddr_Rs, IAddr_Rt;
//    bitset<16> imm;

    //set alu_result to memorize the result of exe stage and send to mem
    bitset<32> alu_result;
//    bool beq_flag = false;

    //alu operation
    bitset<3> add_op(ADDU), subu_op(SUBU);

    //initialize all the stage
    state.IF.nop = false;
    state.ID.nop = true;
    state.EX.nop = true;
    state.EX.alu_op = true;
    state.MEM.nop = true;
    state.WB.nop = true;

    newState.IF.nop = false;
    newState.ID.nop = false;
    newState.EX.nop = false;
    newState.MEM.nop = false;
    newState.WB.nop = false;

    newState.EX.is_I_type = false;
    state.EX.is_I_type = false;
    newState.EX.rd_mem = false;
    state.EX.rd_mem = false;
    newState.EX.wrt_mem = false;
    state.EX.wrt_mem = false;
    state.EX.alu_op = true;
    state.EX.is_beq = false;
    newState.EX.wrt_enable = false;
    state.EX.wrt_enable = false;
    newState.MEM.rd_mem = false;
    state.MEM.rd_mem = false;
    newState.MEM.wrt_mem = false;
    state.MEM.wrt_mem = false;
    newState.MEM.wrt_enable = false;
    state.MEM.wrt_enable = false;
    newState.WB.wrt_enable = false;
    state.WB.wrt_enable = false;
    newState.EX.alu_op = false;
    newState.EX.is_beq = false;


    bitset<32> instruction;
    string op_code, type_function, type_instruction;

    int cycle = 0;
    //R: op[6] rs[5] rt[5] rd[5] shamt[5] func[6]
    //I: op[6] rs[5] rt[5] imm[16]

    //lw MEM(rs+imm) -> rt
    //sw rt -> MEM(rs+imm)
    //beq: if(rs == rt) pc=pc+4+signExtend({imm,00})
    //bne: if(rs != rt) pc=pc+4+signExtend({imm, 00})
    //R addu rs + rt -> rd
    //R subu rd rs rt : rs - rt -> rd

    while (true) {
        /* --------------------- WB stage --------------------- */

        if(state.MEM.nop){
            state.WB.nop = true;
        }else{
            state.WB.nop = false;
            state.WB.Rt = state.MEM.Rt;
            state.WB.Rs = state.MEM.Rs;
            state.WB.Wrt_reg_addr = state.MEM.Wrt_reg_addr;
            state.WB.wrt_enable = state.MEM.wrt_enable;
            state.WB.WB_is_I_type = state.MEM.MEM_is_I_type;
            state.WB.Wrt_data = state.MEM.ALUresult;

            if(state.MEM.wrt_enable && state.WB.WB_is_I_type){ //write data operation for lw instruction
                state.WB.Wrt_data = myDataMem.readDataMem(state.MEM.ALUresult);
                myRF.writeRF(state.WB.Wrt_reg_addr, state.WB.Wrt_data);
            }
            //for R type WB operation
            if(state.MEM.wrt_enable && !state.WB.WB_is_I_type){
                state.WB.Wrt_reg_addr = state.MEM.Wrt_reg_addr;
                myRF.writeRF(state.WB.Wrt_reg_addr, state.WB.Wrt_data);
            }
        }
        newState.WB = state.WB;

        /* --------------------- MEM stage --------------------- */
        if(state.EX.nop){
           state.MEM.nop = true;
        }else{
            state.MEM.nop = false;
            state.MEM.ALUresult = alu_result;
            state.MEM.Rs = state.EX.Rs;
            state.MEM.Rt = state.EX.Rt;
            state.MEM.MEM_is_I_type = state.EX.is_I_type;
            state.MEM.wrt_enable = state.EX.wrt_enable;
            state.MEM.rd_mem = state.EX.rd_mem;//lw
            state.MEM.wrt_mem = state.EX.wrt_mem;//sw

            //sw
            //MEM-MEM forward load-store dependency
            //first judge last instruction is lw type or sw then current instruction is sw
            if(state.WB.WB_is_I_type && state.WB.wrt_enable && !state.MEM.wrt_mem && state.WB.Rt == state.MEM.Rt){
                state.MEM.Store_data = state.WB.Wrt_data;
            }

            //sw
            if(state.MEM.wrt_mem){
                bitset<32> read_Rt = myRF.readRF(state.MEM.Rt);
                //rt->mem(rs+signExtend(imm))[alu_result]
                state.MEM.wrt_enable = false;
                state.MEM.Store_data = read_Rt;
                myDataMem.writeDataMem(alu_result, state.MEM.Store_data);
            }
            //lw read data memory
            if(state.MEM.rd_mem){
                state.MEM.wrt_enable = true;
                state.MEM.Wrt_reg_addr = state.EX.Wrt_reg_addr;
            }

            //R-type
            if(!state.MEM.MEM_is_I_type){
                state.MEM.Wrt_reg_addr = state.EX.Wrt_reg_addr;
            }
        }
        newState.MEM = state.MEM;
        /* --------------------- EX stage --------------------- */

        if(state.ID.nop){
            state.EX.nop = true;
        }else{
            state.EX.nop = false;
            instruction = state.ID.Instr;
            op_code = instruction.to_string().substr(0, 6);
            type_instruction = TypeofInstruction(op_code);
            type_function = TypeofFunction(instruction);

            if(type_instruction == "R"){
                //decode R-type instruction
                RAddresses = OperandsAddr_R(instruction);
                RAddr_Rs = RAddresses[0];
                RAddr_Rt = RAddresses[1];
                RAddr_Rd = RAddresses[2];

                //update state.EX
                state.EX.Read_data1 = myRF.readRF(RAddr_Rs);
                state.EX.Read_data2 = myRF.readRF(RAddr_Rt);
                state.EX.Rs = RAddr_Rs;
                state.EX.Rt = RAddr_Rt;
                state.EX.Wrt_reg_addr = RAddr_Rd;
                state.EX.wrt_enable = true;
                state.EX.is_I_type = false;

                //judge different types of hazards in R type
                //add-add ex-ex forward
                if(!state.MEM.MEM_is_I_type && !state.MEM.nop) {
                    if (state.MEM.Wrt_reg_addr == state.EX.Rs) {
                        state.EX.Read_data1 = state.MEM.ALUresult;
                    }
                    if(state.MEM.Wrt_reg_addr == state.EX.Rt){
                        state.EX.Read_data2 = state.MEM.ALUresult;
                    }
                }

                //add-add independent instruction between mem-ex forward
                if(!state.WB.WB_is_I_type && !state.WB.nop){
                    if(state.WB.Wrt_reg_addr == state.EX.Rs){
                        state.EX.Read_data1 = state.WB.Wrt_data;
                    }

                    if(state.WB.Wrt_reg_addr == state.EX.Rt){
                        state.EX.Read_data2 = state.WB.Wrt_data;
                    }
                }
                //load-add MEM-EX forward
                if(state.WB.WB_is_I_type && !state.WB.nop){
                    if(state.WB.Wrt_reg_addr == state.EX.Rs){
                        state.EX.Read_data1 = state.WB.Wrt_data;
                    }
                    if(state.WB.Wrt_reg_addr == state.EX.Rt){
                        state.EX.Read_data2 = state.WB.Wrt_data;
                    }
                }
                //load-add stall+forward make sure the last instruction is lw
                 if(state.MEM.MEM_is_I_type && !state.MEM.nop && state.MEM.rd_mem){
                    bool stall_Rs = state.EX.Rs == state.MEM.Wrt_reg_addr;
                    bool stall_Rt = state.EX.Rt == state.MEM.Wrt_reg_addr;
                    //stall one cycle and then do forward
                    if(stall_Rs || stall_Rt){
                        state.IF.nop = true;
                        state.EX.nop = true;
                    }
                }

                //if no hazard or we solve all the hazards in R-type instruction, then we can do alu calculation
                if(type_function == "addu"){
                    state.EX.alu_op = true;
                    alu_result = myALU.ALUOperation(add_op, state.EX.Read_data1, state.EX.Read_data2);
                }else if(type_function == "subu"){
                    state.EX.alu_op = false;
                    //hazard，alu_result->$R_Rd
                    alu_result = myALU.ALUOperation(subu_op, state.EX.Read_data1, state.EX.Read_data2);
                }
                state.EX.rd_mem = false;
                state.EX.wrt_mem = false;

            }else if(type_instruction == "lw" || type_instruction == "sw" || type_instruction == "beq") {
                //decode I-type
                IAddressess = OperandsAddr_I(instruction);
                IAddr_Rs = bitset<5>(IAddressess[0]);
                IAddr_Rt = bitset<5>(IAddressess[1]);

                state.EX.Read_data1 = myRF.readRF(IAddr_Rs);
                state.EX.Read_data2 = myRF.readRF(IAddr_Rt);
                state.EX.Imm = bitset<16>(IAddressess[2]);
                state.EX.alu_op = true;
                state.EX.Rs = IAddr_Rs;
                state.EX.Rt = IAddr_Rt;
                state.EX.is_I_type = true;

                /*
                 * after we decode the whole instruction, we can use these parameters to
                 * determine whether there are hazards or not.
                 * */

                //add-load ex-ex forward(last instruction is not I-type)
                if(!state.MEM.MEM_is_I_type && !state.MEM.nop){
                    if(state.MEM.Wrt_reg_addr == state.EX.Rs){
                        state.EX.Read_data1 = state.MEM.ALUresult;
                    }
                }

                //add-load mem-ex
                if(!state.WB.WB_is_I_type && !state.WB.nop){
                    if(state.WB.Wrt_reg_addr == state.EX.Rs){
                        state.EX.Read_data1 = state.WB.Wrt_data;
                    }
                }

                //lw-lw stall+forward(make sure that last instruction is I-type)
                if(state.MEM.MEM_is_I_type && !state.MEM.nop && state.MEM.rd_mem){
                    if(state.MEM.Wrt_reg_addr == state.EX.Rs){
                        state.EX.nop = true;
                        state.IF.nop = true;
                    }
                }

                //lw-lw independent instruction between
                //the instruction before last one is lw
                if(state.WB.WB_is_I_type && !state.WB.nop){
                    if(state.WB.Wrt_reg_addr == state.EX.Rs){
                        state.EX.Read_data1 = state.WB.Wrt_data;
                    }
                }

                /*
                 * after we solve the hazards in I-type instruction,
                 * then we can do alu calculation.
                 * */
                if (type_instruction == "lw") {
                    //lw MEM(rs+imm) -> $I_rt
                    bitset<32> $I_Rs = myRF.readRF(IAddr_Rs);
                    alu_result = myALU.ALUOperation(add_op, $I_Rs, signExtend(state.EX.Imm));
                    state.EX.wrt_enable = true;
                    state.EX.rd_mem = true;
                    state.EX.wrt_mem = false;
                    //myRF.writeRF(IAddr_Rt, $I_Rt);
                    state.EX.Wrt_reg_addr = IAddr_Rt;

                } else if (type_instruction == "sw") {
//                    bitset<32> $I_Rt = myRF.readRF(IAddr_Rt);
                    bitset<32> $I_Rs = myRF.readRF(IAddr_Rs);
                    alu_result = myALU.ALUOperation(add_op, $I_Rs, signExtend(state.EX.Imm));

                    //$Rt -> MEM(alu_result)
                    state.EX.wrt_enable = false;
                    state.EX.rd_mem = false;
                    state.EX.wrt_mem = true;


                } else if (type_instruction == "beq") {
                    //beq if(Rs != Rt) -> branch address
                    //if rs==rt ->pc+4
                    //finalSignExtend rs==rt
                    bitset<32> $beq_Rs = myRF.readRF(IAddr_Rt);
                    bitset<32> $beq_Rt = myRF.readRF(IAddr_Rs);

                    if($beq_Rs == $beq_Rt){
                        //no branch，keep PC+4
                        state.IF.nop = true;
                        state.EX.nop = true;
                        state.EX.is_beq = true;
                    }else{
                        //cal new pc address and set nop
                        string Imm_beq = signExtend(state.EX.Imm).to_string().substr(2, 30) + "00";
                        bitset<32> signExtendImm(Imm_beq);
                        unsigned long counter = programCounter.to_ulong();
                        counter = counter + signExtendImm.to_ulong();
                        //update PC
                        bitset<32> new_beq_pc(counter);
                        programCounter = new_beq_pc;
                        state.IF.nop = true;
                        state.EX.nop = true;
                        state.EX.is_beq = true;
                    }
                }
            }
        }
        newState.EX = state.EX;
        //update the beq_op of newState
        newState.EX.is_beq = false;

        /* --------------------- ID stage --------------------- */
        if(state.IF.nop){
            if(state.EX.is_beq)
                state.ID.nop = true;
            newState.ID = state.ID;
        }else{
            state.ID.nop = false;
            state.ID.Instr = myInsMem.readInstr(programCounter);
            newState.ID = state.ID;
        }

        /* --------------------- IF stage --------------------- */
        if(state.IF.nop){
            newState.IF = state.IF;
            newState.IF.nop = false;//set back and keep the pc run
        }else{
            state.IF.nop = false;
            state.IF.PC = programCounter;
            newState.IF.PC = calProgramCounter(programCounter);
        }

        //halt
        if(state.ID.Instr.to_string() == "11111111111111111111111111111111"){
            state.IF.nop = true;
            state.ID.nop = true;
            newState.IF.nop = state.IF.nop;
            newState.ID.nop = state.ID.nop;
        }

        printState(newState, cycle); //print states after executing cycle 0, cycle 1, cycle 2 ...

        if (state.IF.nop && state.ID.nop && state.EX.nop && state.MEM.nop && state.WB.nop)
            break;

        state = newState; /*The end of the cycle and updates the current state with the values calculated in this cycle */
        cycle++;
    }

    myRF.outputRF(); // dump RF;
    myDataMem.outputDataMem(); // dump data mem
    return 0;
}