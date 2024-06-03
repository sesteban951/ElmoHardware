#include "../inc/ElmoComm.hpp"

/* ELMO order of joints (physical daisy chain order)
  1. HFL  (Hip Frontal Left)
  2. HSL  (Hip Sagittal Left)
  3. HSR  (Hip Sagittal Right)
  4. KL   (Knee Left)
  5. HFR  (Hip Frontal Right)
  6. KR   (Knee Right)
*/ 

#define EC_TIMEOUTMON 500

char IOmap[4096];
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;


// **************************************************************************************************************************


// Service Data Object (SDO) READ macro. 
#define READ(slaveId, idx, sub, buf, comment)    \
    {   \
        buf=0;  \
        int __s = sizeof(buf);    \
        int __ret = ec_SDOread(slaveId, idx, sub, FALSE, &__s, &buf, EC_TIMEOUTRXM);   \
        printf("Slave: %d - Read at 0x%04x:%d => wkc: %d; data: 0x%.*x (%d)\t[%s]\n", slaveId, idx, sub, __ret, __s,(unsigned int)buf, (unsigned int)buf, comment);    \
    }

// Service Data Object (SDO) WRITE macro
#define WRITE(slaveId, idx, sub, buf, value, comment) \
    {   \
        int __s = sizeof(buf);  \
        buf = value;    \
        int __ret = ec_SDOwrite(slaveId, idx, sub, FALSE, __s, &buf, EC_TIMEOUTRXM);  \
        printf("Slave: %d - Write at 0x%04x:%d => wkc: %d; data: 0x%.*x\t{%s}\n", slaveId, idx, sub, __ret, __s, (unsigned int)buf, comment);    \
    }

// Check for errors macro
#define CHECKERROR(slaveId)   \
{   \
    ec_readstate();\
    printf("EC> \"%s\" %x - %x [%s] \n", (char*)ec_elist2string(), ec_slave[slaveId].state, ec_slave[slaveId].ALstatuscode, (char*)ec_ALstatuscode2string(ec_slave[slaveId].ALstatuscode));    \
}


// **************************************************************************************************************************

// ELMO communication function. Setup and stream data
void *ELMOcommunication(void *data) {

    // useful variables
    int i, chk;
    needlf = FALSE;
    inOP = FALSE;
    uint32 buf32;
    uint16 buf16;
    uint8 buf8;
    ELMOData * data_pointer;

    // Funky pointer stuff to cast void* data correctly
    data_pointer = *(ELMOData **) data; // cast the void pointer to a ELMOData pointer
    char ifname[1028];                  // ethernet port name container
    strcpy(ifname,data_pointer->port);  // copy the port name to the container

    // ELMO structs
    struct ELMOIn *val[6];              // ELMO --> Laptop
    struct ELMOOut *target[6];          // Laptop --> ELMO
    
    printf("Starting ELMO communication\n");

    /* initialise SOEM, bind socket to ifname */
    if (ec_init(ifname))
    {
        // if we was able to bind the socket print success message
        printf("ec_init on %s succeeded.\n",ifname);

        /* find and auto-config slaves */

        /** network discovery */
        if ( ec_config_init(FALSE) > 0 )
        {
            printf("%d slaves found and configured.\n", ec_slavecount);

            for (int i=1; i<=ec_slavecount; i++) {
                printf("Slave %d has CA? %s\n", i, ec_slave[i].CoEdetails & ECT_COEDET_SDOCA ? "true":"false" );

                /** CompleteAccess disabled for Elmo driver */
                ec_slave[i].CoEdetails ^= ECT_COEDET_SDOCA;
            }
            
            ec_statecheck(0, EC_STATE_PRE_OP,  EC_TIMEOUTSTATE);

            /** opMode: 8   => Cyclic position 
                opMode: 10  => Cyclic Synchronous Torque */
            uint8 opmode = data_pointer->OpMode;
            for (int i=1; i<=ec_slavecount; i++) {

                // Operation Mode
                WRITE(i, 0x6060, 0, buf8, opmode, "OpMode"); // <--- TODO: resolve 
                                                             // having to change this to 8 and then 10 to work

                // Operation Mode Display
                READ(i, 0x6061, 0, buf8, "OpMode display");

                READ(i, 0x1c12, 0, buf32, "rxPDO:0");
                READ(i, 0x1c13, 0, buf32, "txPDO:0");

                READ(i, 0x1c12, 1, buf32, "rxPDO:1");
                READ(i, 0x1c13, 1, buf32, "txPDO:1");
            }
            
            /** set PDO mapping */
            int32 ob2;int os;
            for (int i=1; i<=ec_slavecount; i++) {                

                //  set to 'Target Torque'
                os=sizeof(ob2); ob2 = 0x16020001;            
                ec_SDOwrite(i, 0x1c12, 0, TRUE, os, &ob2, EC_TIMEOUTRXM);
                
                //  set to 'Position/Velocity Actual Values'
                os=sizeof(ob2); ob2 = 0x1a030001;             
                ec_SDOwrite(i, 0x1c13, 0, TRUE, os, &ob2, EC_TIMEOUTRXM);

                READ(i, 0x1c12, 0, buf32, "rxPDO:0");
                READ(i, 0x1c13, 0, buf32, "txPDO:0");

                READ(i, 0x1c12, 1, buf32, "rxPDO:1");
                READ(i, 0x1c13, 1, buf32, "txPDO:1");
            }
            
            /** if CA disable => automapping works */
            ec_config_map(&IOmap); 
            ec_configdc(); 

            // show slave info
            for (int i=1; i<=ec_slavecount; i++) {
                printf("\nSlave:%d\n Name:%s\n Output size: %dbits\n Input size: %dbits\n State: %d\n Delay: %d[ns]\n Has DC: %d\n",
                i, ec_slave[i].name, ec_slave[i].Obits, ec_slave[i].Ibits,
                ec_slave[i].state, ec_slave[i].pdelay, ec_slave[i].hasdc);
            }

            /** disable heartbeat alarm */
            for (int i=1; i<=ec_slavecount; i++) {
                READ(i, 0x10F1, 2, buf32, "Heartbeat?");
                WRITE(i, 0x10F1, 2, buf32, 1, "Heartbeat");

                WRITE(i, 0x60c2, 1, buf8, 2, "Time period");
                WRITE(i, 0x2f75, 0, buf16, 2, "Interpolation timeout");
            }           

            printf("Slaves mapped, state to SAFE_OP.\n");

            /* wait for all slaves to reach SAFE_OP state */
            ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);

            printf("segments : %d : %d %d %d %d\n",ec_group[0].nsegments ,ec_group[0].IOsegment[0],ec_group[0].IOsegment[1],ec_group[0].IOsegment[2],ec_group[0].IOsegment[3]);

            printf("Request operational state for all slaves\n");
            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            printf("Calculated workcounter %d\n", expectedWKC);

            /** going operational */
            ec_slave[0].state = EC_STATE_OPERATIONAL;

            /* send one valid process data to make outputs in slaves happy*/
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);

            // see what the max and min acceleration and deceleration values are set to
            for (int i=1; i<=ec_slavecount; i++) {
                READ(i, 0x6083, 0, buf32, "Profile acceleration");    // read and it says (1)
                READ(i, 0x6084, 0, buf32, "Profile deceleration");    // read and it says (1)
                READ(i, 0x6085, 0, buf32, "Quick stop deceleration"); // read and it says (1)
            }
            
            /* request OP state for all slaves */
            ec_writestate(0);
            chk = 40;
            
            /* wait for all slaves to reach OP state */
            do
            {
                ec_send_processdata();
                ec_receive_processdata(EC_TIMEOUTRET);
                ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
            }
            while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

            if (ec_slave[0].state == EC_STATE_OPERATIONAL )
            {
                printf("Operational state reached for all slaves.\n");
                inOP = TRUE;

                /**
                 * Drive state machine transitions
                 *   0 -> 6 -> 7 -> 15
                 */
                for (int i=1; i<=ec_slavecount; i++) {
                    READ(i, 0x6041, 0, buf16, "*status word*");
                    if(buf16 == 0x218)
                    {
                        WRITE(i, 0x6040, 0, buf16, 128, "*control word*"); usleep(100000);
                        READ(i, 0x6041, 0, buf16, "*status word*");
                    }

                    WRITE(i, 0x6040, 0, buf16, 0, "*control word*"); usleep(100000);
                    READ(i, 0x6041, 0, buf16, "*status word*"); 

                    WRITE(i, 0x6040, 0, buf16, 6, "*control word*"); usleep(100000);
                    READ(i, 0x6041, 0, buf16, "*status word*"); 

                    WRITE(i, 0x6040, 0, buf16, 7, "*control word*"); usleep(100000);
                    READ(i, 0x6041, 0, buf16, "*status word*"); 

                    WRITE(i, 0x6040, 0, buf16, 15, "*control word*"); usleep(100000);
                    READ(i, 0x6041, 0, buf16, "*status word*"); 

                    CHECKERROR(i);
                    READ(i, 0x1a0b, 0, buf8, "OpMode Display");

                    READ(i, 0x1001, 0, buf8, "Error");
                }   

                // assign the ElmoIn and ElmoOut structs to each ELMO motor controller
                /* cyclic loop for all slaves, "j+1" b/c slaves are 1-indexed*/
                for (int j = 0; j <ec_slavecount; j++) {

                  target[j] = (struct ELMOOut *)(ec_slave[j+1].outputs); // data struct to send to ELMO    
                  val[j] = (struct ELMOIn *)(ec_slave[j+1].inputs);      // data struct to receive from ELMO
                }

                //----------------------------------------- MAIN LOOP ------------------------------------------

                // set the communication status to operating
                data_pointer->commStatus = 1;

                // for maintaining the loop frequency
                auto t1 = std::chrono::high_resolution_clock::now();
                double dt = 0.0;

                // main loop
                while(1) {

                    // check if the motor state is switched ot off
                    if (data_pointer->motor_control_switch == false) {
                        break;
                    }

                    // for maintaining the loop frequency
                    auto t2 = std::chrono::high_resolution_clock::now();
                    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
                    dt = duration.count() / 1'000'000.0; // convert to seconds
                    
                    // execute this block of code every 1/freq seconds
                    if (dt >= (1.0 / data_pointer->freq)) {

                        // update the time
                        t1 = t2;

                        /** PDO I/O refresh */
                        ec_send_processdata();
                        wkc = ec_receive_processdata(EC_TIMEOUTRET);

                        if(wkc >= expectedWKC) {

                            // update the data pointer with newest ELMO encoder data
                            for (int j = 0; j < ec_slavecount; j++) {

                                // update torque sent to ELMO
                                data_pointer->torque[j] = target[j]->torque;
                                
                                // update encoder data
                                data_pointer->pos[j] = val[j]->position;  
                                data_pointer->vel[j] = val[j]->velocity;

                                // update diagnostic data
                                data_pointer->inputs[j] = val[j]->inputs;
                                data_pointer->controlword[j] = target[j]->controlword; 
                                data_pointer->statusword[j] = val[j]->status;
                            }
                            
                            // std::cout << "-----------------------------------" << std::endl;
                            
                            for (int i = 0; i < ec_slavecount; i++)  {        

                                // Do it for the first elmo slave
                                uint16_t ctrlWord_tmp = 0;
                                uint16 statusWord = val[i]->status;
                                // std::cout << "\nStatus "<< i+1 <<": " << statusWord << std::endl;

                                if(!((statusWord >> 0) & 1) && !((statusWord >> 1) & 1) && !((statusWord >> 2) & 1) && !((statusWord >> 3) & 1) && !((statusWord >> 6) & 1)){    
                                    // NOT READY
                                    // std::cout << "Drive " << i+1 <<" NOT ready." << std::endl;
                                    target[i]->torque = (int16) 0;
                                } 
                                else if (!((statusWord >> 0) & 1) && !((statusWord >> 1) & 1) && !((statusWord >> 2) & 1) && !((statusWord >> 3) & 1) && ((statusWord >> 6) & 1)){
                                    // SOD
                                    std::cout << "Drive " << i+1 << " is in SOD." << std::endl;
                                    ctrlWord_tmp = (1 << 1) | ctrlWord_tmp; 
                                    ctrlWord_tmp = (1 << 2) | ctrlWord_tmp;    
                                    target[i]->controlword = ctrlWord_tmp; 
                                    // std::cout << "Control word: " << target[i]->controlword << std::endl;
                                    target[i]->torque = (int16) 0;
                                }
                                else if (((statusWord >> 0) & 1) && !((statusWord >> 1) & 1) && !((statusWord >> 2) & 1) && !((statusWord >> 3) & 1) && ((statusWord >> 5) & 1) && !((statusWord >> 6) & 1)) {
                                    // RSO
                                    std::cout << "Drive " << i+1 << " is in RSO." << std::endl;
                                    ctrlWord_tmp = (1 << 0) | ctrlWord_tmp; 
                                    ctrlWord_tmp = (1 << 1) | ctrlWord_tmp; 
                                    ctrlWord_tmp = (1 << 2) | ctrlWord_tmp; 
                                    target[i]->controlword = ctrlWord_tmp; 
                                    // std::cout << "Control word: " << target[i]->controlword << std::endl;
                                    target[i]->torque = (int16) 0;
                                }
                                else if (((statusWord >> 0) & 1) && ((statusWord >> 1) & 1) && !((statusWord >> 2) & 1) && !((statusWord >> 3) & 1) && ((statusWord >> 5) & 1) && !((statusWord >> 6) & 1)) {
                                    // SO
                                    std::cout << "Drive " << i+1 << " in SO." << std::endl;
                                    ctrlWord_tmp = (1 << 0) | ctrlWord_tmp; 
                                    ctrlWord_tmp = (1 << 1) | ctrlWord_tmp; 
                                    ctrlWord_tmp = (1 << 2) | ctrlWord_tmp; 
                                    ctrlWord_tmp = (1 << 3) | ctrlWord_tmp; 
                                    target[i]->controlword = ctrlWord_tmp;
                                    // std::cout << "Control word: " << target[i]->controlword << std::endl;
                                    target[i]->torque = (int16) 0;
                                }
                                else if (((statusWord >> 0) & 1) && ((statusWord >> 1) & 1) && ((statusWord >> 2) & 1) && !((statusWord >> 3) & 1) && ((statusWord >> 5) & 1) && !((statusWord >> 6) & 1)) {
                                    // ARMED
                                    std::cout << "Drive " << i+1 << " ARMED." << std::endl;
                                    ctrlWord_tmp = (1 << 0) | ctrlWord_tmp; 
                                    ctrlWord_tmp = (1 << 1) | ctrlWord_tmp; 
                                    ctrlWord_tmp = (1 << 2) | ctrlWord_tmp; 
                                    ctrlWord_tmp = (1 << 3) | ctrlWord_tmp;
                                    target[i]->controlword = ctrlWord_tmp;
                                    std::cout << "Control word: " << target[i]->controlword << std::endl;

                                    // Apply the desired torque
                                    target[i]->torque = data_pointer->torque[i];
                                }
                                else if (((statusWord >> 0) & 1) && ((statusWord >> 1) & 1) && ((statusWord >> 2) & 1) && !((statusWord >> 3) & 1) && !((statusWord >> 5) & 1) && !((statusWord >> 6) & 1)){
                                    // FAILED
                                    // std::cout << "Drive " << i+1 << " QUICK STOPPED." << std::endl;
                                    target[i]->torque = (int16) 0;
                                }
                                else if (((statusWord >> 0) & 1) && ((statusWord >> 1) & 1) && ((statusWord >> 2) & 1) && ((statusWord >> 3) & 1) && !((statusWord >> 6) & 1)){
                                    // FAILED
                                    // std::cout << "Drive " << i+1 << " FAILED: fault reaction active." << std::endl;
                                    target[i]->torque = (int16) 0;
                                }
                                else if (!((statusWord >> 0) & 1) && !((statusWord >> 1) & 1) && !((statusWord >> 2) & 1) && ((statusWord >> 3) & 1) && !((statusWord >> 6) & 1)){
                                    // FAILED
                                    // std::cout << "Drive " << i+1 << " FAILED: fault." << std::endl;
                                    target[i]->torque = (int16) 0;
                                }
                                else{
                                    // unknown state
                                    // std::cout << "Drive " << i+1 << " FAILED: unknown state." << std::endl;
                                    target[i]->torque = (int16) 0;
                                }

                                // send the control word to the ELMO based on what status word was read
                                target[i]->controlword = ctrlWord_tmp;
                            }
                        }
                        needlf = TRUE;
                    }
                    usleep(10);
                }

                //----------------------------------------- SHUTDOWN ------------------------------------------
                
                std::cout << "-----------------------------------" << std::endl;
                
                if (data_pointer->motor_control_switch == false) {
                    
                    // send zero torque and status word to all motors
                    for (int i=0; i<ec_slavecount; i++) {
                        target[i]->torque = (int16) 0;
                        target[i]->controlword = 0;
                    }
                    usleep(500);

                    // check the status of the motors
                    for (int i=0; i<ec_slavecount; i++) {

                        uint16 statusWord = val[i]->status;
                        std::cout << "\nStatus " << i+1 <<": " << statusWord << std::endl;

                        if (((statusWord >> 0) & 1) && ((statusWord >> 1) & 1) && ((statusWord >> 2) & 1) && !((statusWord >> 3) & 1) && ((statusWord >> 5) & 1) && !((statusWord >> 6) & 1)) {
                            // SHUTDOWN
                            std::cout << "Drive " << i+1 << " is SHUTDOWN." << std::endl;
                        }
                        else {
                            // NOT ARMED
                            std::cout << "Drive " << i+1 << " was not ARMED." << std::endl;
                        }
                    }
                }
                
                inOP = FALSE;
            }
            else
            {
                printf("Not all slaves reached operational state.\n");
                ec_readstate();
                for(i = 1; i<=ec_slavecount ; i++)
                {
                    if(ec_slave[i].state != EC_STATE_OPERATIONAL)
                    {
                        printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                            i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                    }
                }
            }

            printf("\nRequest init state for all slaves\n");
            for (int i=1; i<=ec_slavecount; i++) {
                WRITE(i, 0x10F1, 2, buf32, 0, "Heartbeat");
            }
           
            ec_slave[0].state = EC_STATE_INIT;
            /* request INIT state for all slaves */
            ec_writestate(0);
        }
        else
        {
            printf("No slaves found!\n");
        }
        printf("End simple test, close socket\n");
        
        /* stop SOEM, close socket */
        ec_close();
        data_pointer->commStatus = -1;
    }
    else
    {
        printf("No socket connection on %s\nExcecute as root\n",ifname);
    }
}

// **************************************************************************************************************************


// callback function to check for ethercat comm errors
void *ecatcheck(void* why) {

    int slave;

    while(1)
    {
        if( inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
        {
            if (needlf)
            {
               needlf = FALSE;
               printf("\n");
            }
            /* one ore more slaves are not responding */
            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();
            for (slave = 1; slave <= ec_slavecount; slave++)
            {
               if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
               {
                  ec_group[currentgroup].docheckstate = TRUE;
                  if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                  {
                     printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                     ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                     ec_writestate(slave);
                  }
                  else if(ec_slave[slave].state == EC_STATE_SAFE_OP)
                  {
                    printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                    ec_slave[slave].state = EC_STATE_OPERATIONAL;
                    ec_writestate(slave);
                  }
                  else if(ec_slave[slave].state > 0)
                  {
                     if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                     {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d reconfigured\n",slave);
                     }
                  }
                  else if(!ec_slave[slave].islost)
                  {
                     /* re-check state */
                     ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                     if (!ec_slave[slave].state)
                     {
                        ec_slave[slave].islost = TRUE;
                        printf("ERROR : slave %d lost\n",slave);
                     }
                  }
               }
               if (ec_slave[slave].islost)
               {
                  if(!ec_slave[slave].state)
                  {
                     if (ec_recover_slave(slave, EC_TIMEOUTMON))
                     {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d recovered\n",slave);
                     }
                  }
                  else
                  {
                     ec_slave[slave].islost = FALSE;
                     printf("MESSAGE : slave %d found\n",slave);
                  }
               }
            }
            if(!ec_group[currentgroup].docheckstate)
               printf(".");
        }
        usleep(1000);
    }
}
