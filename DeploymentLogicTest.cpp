//this is currently done with pressure so let's incorporate the other thing 

#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <iomanip>
#include <vector>
#include <array>


//using namespace std;


//use commas as a delimiters

//A vector is specifically a series of elements that indicate some multi-dimensional thing like speed or a point in space 
enum stages  
{
    READY,
    FIRST_STAGE_FLIGHT,
    FIRST_STAGE_COASTING,
    APOGEE,
    // SECOND_STAGE_FLIGHT,
    // SECOND_STAGE_COASTING,
    PARACHUTE_DEPLOYED,
    LANDED,
    EXPLODED,
};

int current_stage = READY; 


//defining the values in the packet
typedef struct 
{
    long time; //4 bytes - 4
    float error;// 4 bytes - 8
    //int code; // 10 some of these values are not in csv
    float voltage; // 4 bytes - 12
    float rawx; // 4 bytes - 16
    float rawy; // 4 bytes - 20 
    float rawz; // 4 bytes - 24
    float accx; // 4 bytes - 28
    float accy; // 4 bytes - 32
    float accz; // 4 bytes - 36
    float avelx; // 4 bytes - 40
    float avely; // 4 bytes - 44
    float avelz; // 4 bytes - 48
    float magx; // 4 bytes - 52
    float magy; // 4 bytes - 56
    float magz; // 4 bytes - 60
    float altitude; // 4 bytes - 64
    float temp; // 4 bytes - 68
    //float pressure;
    float w; // 4 bytes - 72 (quaternion nonsense)
    float x; // 4 bytes - 76
    float y; // 4 bytes - 80
    float z; // 4 bytes - 84
    long long checksum; // >:C   8 bytes - 92
} Packet; //22 values



//function definitions:
Packet extract_values(std::string values);
std::vector<Packet> packet_maker(int* num_packets, std::vector<int>* line_breaks); 
// ^ look at address of numpackets - modifies oG variable it pointing to 
//line breaks is the position of line breaks

float accessPacketFloat(Packet packet, int index);
//vector refers to a library 

//************************************************************* MAIN  *********************************
int main() 
{


//vector of packets (basically an array but not really)
    std::vector<Packet> packet_vector;
    std::vector<int> line_breaks;
    
    //ifstream copyData;
    std::ofstream outData;
    std::string values = "";
    
    int num_packets = 0;

    packet_vector = packet_maker(&num_packets, &line_breaks);

#define abcd packet_vector[num_packets] //abcd = packet_vectors[num_packets]
    std::cout << "num_packets: " << num_packets + 1 << "  last quaternion: " << abcd.w << " " << abcd.x << " " << abcd.y << " " << abcd.z << " " << std::endl;
    std::cout << "some checksums: " << abcd.checksum << " " << packet_vector[num_packets - 1].checksum << " " << packet_vector[num_packets - 2].checksum << " " << std::endl;

    //setup done, time for the rocket simulation


    constexpr int packet_mem_size = 60; //literally a constant. why are we so dramatic 'constexpr?
    //packet memory size  = packet_mem_size
    //there are 60 packets. everytime a new apcket is introduced the other is kicked out

    std::array<Packet, packet_mem_size> packet_memory;
    Packet rolling_integral = packet_vector[0]; //rolling integral - changes as time passes (not in use rn)
    Packet rolling_derivative = packet_vector[0]; // 0 means we getting intiaitve val
    std::array<float, packet_mem_size> deriv_altitude;
    std::array<float, packet_mem_size> deriv_accz;

    long delta_time  = 0;

    //sets up the packet memory with first packet_mem_size packets (initializes the packets)
    packet_memory[0] = packet_vector[0];
    deriv_altitude[0] = 0;
    deriv_accz[0]=0;
    for (int i = 1; i < packet_mem_size; i++) 
    {
        packet_memory[i] = packet_vector[i];
        delta_time = (packet_memory[i].time - packet_memory[i-1].time)/1000000;

       // deriv_pressure[i] = (packet_memory[i].pressure - packet_memory[i - 1].pressure) * ((packet_memory[i].delta_time)/1000000);
        deriv_altitude[i] = (packet_memory[i].altitude - packet_memory[i - 1].altitude) * (delta_time);
        deriv_accz[i] = (packet_memory[i].accz - packet_memory[i - 1].accz) * (delta_time);

    }


     //the code that does. it do be do
    //tick is number of program cycles that have passed



    int current_stage = READY;
    for (int tick = packet_mem_size; tick < num_packets; tick++) 
    {
        delta_time = (packet_memory[tick].time - packet_memory[tick-1].time)/1000000;
        int memory_index = tick % packet_mem_size; //modulo my beloved
        packet_memory[memory_index] = packet_vector[tick]; //set up packet from outside data
        deriv_altitude[memory_index] = (packet_memory[memory_index].altitude - packet_memory[(tick - 1) % packet_mem_size].altitude) * delta_time;
        deriv_accz[memory_index] = (packet_memory[memory_index].accz - packet_memory[(tick - 1) % packet_mem_size].accz) * delta_time;

        float avg_deriv_altitude;
        float altitude_sum = 0;
        for (int k = 0; k < packet_mem_size; k++)
        {
            altitude_sum += deriv_altitude[k]; //the diffential between two packets could be 0 
            //so we do it thoruh the whole 60 
        }
        altitude_sum = altitude_sum / (float)packet_mem_size;
        std::cout << altitude_sum << "       " << (packet_memory[memory_index].time);

    switch (current_stage)
    {
        case READY: 
        {
            if(deriv_altitude[memory_index] > 1)  //value isn't perfect. BUt 1 is significant enough to indicate the enigine fired
            {
                current_stage = FIRST_STAGE_FLIGHT;
                std::cout << "rocket go woosh (engine fired) ";
            }
        } break;
        case FIRST_STAGE_FLIGHT: 
        {
            if(deriv_accz[memory_index] <= -1)  
            {
                current_stage = FIRST_STAGE_COASTING;
                std::cout << "coasting boastin ";
            }
        } break;
        case FIRST_STAGE_COASTING: 
        {
            if(deriv_accz[memory_index] >=-0.1 && deriv_accz[memory_index] <= 0.1  )  
            {
                current_stage = APOGEE;
                std::cout << "reached max";
            }
        } break;
        case APOGEE: 
        {
           if(deriv_altitude[memory_index] < -0.1 )  
            {
                current_stage = PARACHUTE_DEPLOYED;
                std::cout << "parachute deployed BOOM";
            }
        } break;
        case PARACHUTE_DEPLOYED:
        {
            if(deriv_altitude[memory_index] < -0.1 && (deriv_accz[memory_index] <= 0 && deriv_accz[memory_index] >= -0.5 ))  // very little fluctuation
            {
                current_stage = LANDED;
                std::cout << "we gournd";
            }
        } break;
        
        default:
            break;
    }
        std::cout << std::endl;
        //if (deriv_pressure[memory_index] != 0) std::cout << deriv_pressure[memory_index] << "       " << (packet_memory[memory_index].time) << std::endl;

    }
    
    
    
}
//main ends---

//adding sustinence to the functions
//the append thing is cuz it if runs out it breaks 
Packet extract_values(std::string values) {
    //cout << values << endl;
    values.append(",123"); // never thought i would have one of those "add this weird thing so it doesnt break" moments but yet here we are
    Packet ret; //return 
    int position = 0;
    for (int i = 1; i < 18; i++) 
    {

        //cout << "value position: " << i << endl;
        std::string temp_value = "";
        for (int j = position; j <= values.length(); j++) 
        {
            position = j + 1;
            //cout << "value position: " << i << "  string position: " << j <<"  position: " << position << "  temp value: " << temp_value << endl;
            if (values[j] == ',') {
                //cout << "ugh" << endl;
                switch (i) 
                {
                    case 1: {
                        //cout << temp_value << endl;
                        ret.error = stof(temp_value);  //string to float 
                        //cout << "yay?" << endl;
                    } break;
                    case 2: {
                        ret.time = stol(temp_value); //string to long
                    } break;
                    case 3: {
                        ret.time = stol(temp_value); //changed delta_time to time. they aren't exactly same tho i should make a delta time right
                    } break;
                    case 4: {
                        ret.accx = stof(temp_value);
                    } break;
                    case 5: {
                        ret.accy = stof(temp_value);
                    } break;
                    case 6: {
                        ret.accz = stof(temp_value);
                    } break;
                    case 7: {
                        ret.avelx = stof(temp_value);
                    } break;
                    case 8: {
                        ret.avely = stof(temp_value);
                    } break;
                    case 9: {
                        ret.avelz = stof(temp_value);
                    } break;
                    case 10: {
                        ret.altitude = stof(temp_value);
                    } break;
                    // case 11: {
                    //     ret.pressure = stof(temp_value);
                    // } break;
                    case 11: {
                        ret.temp = stof(temp_value);
                    } break;
                    case 12: {
                        ret.w = stof(temp_value);
                    } break;
                    case 13: {
                        ret.x = stof(temp_value);
                    } break;
                    case 14: {
                        ret.y = stof(temp_value);
                    } break;
                    case 15: {
                        ret.z = stof(temp_value);
                    } break;
                    case 16: {
                        //cout << temp_value << endl;
                        ret.checksum = stoll(temp_value);
                        //cout << (unsigned int)stoi(temp_value) << endl;
                    } break;
                }
                break;
            }
            else {
                temp_value.push_back(values[j]);
                //cout << "nonse" << endl;
            }
        }
    }
    return ret;
}

std::vector<Packet> packet_maker(int* num_packets, std::vector<int>* line_breaks) {

    std::ifstream inData;
    inData.open("urrg.csv");

    char c = inData.get();
    std::vector<Packet> packet_vector;
    int position = 0;
    bool notfirst = false;
    std::string values = "";

    while (inData.good()) {
        if (c == '\n') {
            line_breaks->push_back(position); //line_breaks.pushback but since line_breaks is a pointer use ->
            //cout << "position: " << position << endl;
            if (notfirst == false) {
                values = "";
                notfirst = true;
            }
            else {
                //cout << "values: " << values << endl;
                (*num_packets)++;
                packet_vector.push_back(extract_values(values));
                values = "";
            }
            values = "";
        }
        else {
            values.push_back(c);
        }
        //if (num_packets >= 10) break;
        position++;
        c = inData.get();
    } //*/
    (*num_packets)--;

    return packet_vector;

}

//to make it easier to iterate through the packet vector (may not actually be easy)
float accessPacketFloat(Packet packet, int index) { //this is ass but i dont know how to make function pointers and i dont want to figure it out right now
    float ret;
    switch (index + 1) {
    case 1: {
        ret = (float)packet.time;
    } break;
    case 2: {
        ret = packet.error;
    } break;
    case 3: {
        ret = packet.voltage;
    } break;
    case 4: {
        ret = packet.rawx;
    } break;
    case 5: {
        ret = packet.rawy;
    } break;
    case 6: {
        ret = packet.rawz;
    } break;
    case 7: {
        ret = packet.accx;
    } break;
    case 8: {
        ret = packet.accy;
    } break;
    case 9: {
        ret = packet.accz;
    } break;
    case 10: {
        ret = packet.avelx;
    } break;
    case 11: {
        ret = packet.avely;
    } break;
    case 12: {
        ret = packet.avelz;
    } break;
    case 13: {
        ret = packet.magx;
    } break;
    case 14: {
        ret = packet.magy;
    } break;
    case 15: {
        ret = packet.magz;
    } break;
    case 16: {
        ret = packet.altitude;
    } break;
    case 17: {
        ret = packet.temp;
    } break;
    case 18: {
        ret = packet.w;
    } break;
    case 19: {
        ret = packet.x;
    } break;
    case 20: {
        ret = packet.y;
    } break;
    case 21: {
        ret = packet.z;
    } break;
    default:
        ;//index out of bounds (only allows 0-15)
    }
    return ret;
}