
#include "../inc/ElmoComm.h"

#define actuator_conversion_factor 0.0001

#define KNEEMIN -0.01
#define KNEEMAX  0.01

#define HIPMIN -0.01
#define HIPMAX  0.01

#define EC_TIMEOUTMON 500

programState progState;

char IOmap[4096];
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;
char usdo[128];
char hstr[1024];

// **************************************************************************************************************************//

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


// **************************************************************************************************************************//

// ELMO communication function. Setup and stream data
void *ELMOcommunication(void *data) {
    
    int i, chk;
    needlf = FALSE;
    inOP = FALSE;
    uint32 buf32;
    uint16 buf16;
    uint8 buf8;
    ELMOData * data_pointer;

    // Funky pointer stuff to cast void* data correctly
    data_pointer = *(ELMOData **) data;
    char ifname[1028];
    strcpy(ifname,data_pointer->port);
    struct ELMOIn *val[6];
    struct ELMOOut *target[6];
    
    printf("Starting ELMO communication\n");

    /* initialise SOEM, bind socket to ifname */
    if (ec_init(ifname))
    {
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

            /** opMode: 8  => Position profile */
            for (int i=1; i<=ec_slavecount; i++) {
                WRITE(i, 0x6060, 0, buf8, 10, "OpMode");
                READ(i, 0x6061, 0, buf8, "OpMode display");


                READ(i, 0x1c12, 0, buf32, "rxPDO:0");
                READ(i, 0x1c13, 0, buf32, "txPDO:0");

                READ(i, 0x1c12, 1, buf32, "rxPDO:1");
                READ(i, 0x1c13, 1, buf32, "txPDO:1");
            }

            /** set PDO mapping */
            int32 ob2;int os;
            for (int i=1; i<=ec_slavecount; i++) {                
                os=sizeof(ob2); ob2 = 0x16020001;
                ec_SDOwrite(i, 0x1c12, 0, TRUE, os, &ob2, EC_TIMEOUTRXM);
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

            for (int i=1; i<=ec_slavecount; i++) {
                READ(i, 0x6083, 0, buf32, "Profile acceleration");
                READ(i, 0x6084, 0, buf32, "Profile deceleration");
                READ(i, 0x6085, 0, buf32, "Quick stop deceleration");
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
                int reachedInitial[] = {0,0,0,0,0,0};        

                /* cyclic loop for two slaves*/
                for (int j = 0; j <ec_slavecount; j++) {
                  target[j] = (struct ELMOOut *)(ec_slave[j+1].outputs);
                  val[j] = (struct ELMOIn *)(ec_slave[j+1].inputs);
                }
                // Ready
                data_pointer->commStatus = 1;
                while(1)
                {
                    /** PDO I/O refresh */
                    ec_send_processdata();
                    wkc = ec_receive_processdata(EC_TIMEOUTRET);

                    if(wkc >= expectedWKC) {
                        for (int j =0; j < ec_slavecount; j++) {
                          /** if in fault or in the way to normal status, we update the state machine */
                          switch(target[j]->controlword){
                            case 0:
                                target[j]->controlword = 6; // Enable voltage, quick stop
                                break;
                            case 6:
                                target[j]->controlword = 7; // switch on
                                break;
                            case 7:
                                target[j]->controlword = 15; // enable operation
                                break;
                            case 128:
                                target[j]->controlword = 0; // reset
                                break;
                            default:
                                if(val[j]->status >> 3 & 0x01) {
                                    READ(1, 0x1001, 0, buf8, "Error");
                                    target[j]->controlword = 128; // Halt
                                }
                          }
                          if(reachedInitial[j] == 0 && (val[j]->status & 0x0fff) == 0x0237){
                              reachedInitial[j] = 1;
                          }
                        }

                        data_pointer->torso = val[5]->position;
                        data_pointer->torso_d = val[5]->velocity;

                        for (int j = 0; j < 4; j++) {
                          data_pointer->pos[j] = val[j+1]->position;  
                          data_pointer->vel[j] = val[j+1]->velocity;

                          switch (j) {
                            case 0:
                              if ((data_pointer->pos[j]*actuator_conversion_factor > KNEEMAX && data_pointer->torque[j] > 0) | (data_pointer->pos[j]*actuator_conversion_factor < KNEEMIN && data_pointer->torque[j] < 0)) {
                                data_pointer->torque[j]=0;
                                std::cout << "Warning: Knee joint limit reached" << std::endl;
                              }
                            break;
                            case 3:
                              if ((data_pointer->pos[j]*actuator_conversion_factor > KNEEMAX && data_pointer->torque[j] > 0) | (data_pointer->pos[j]*actuator_conversion_factor < KNEEMIN && data_pointer->torque[j] < 0)) {
                                data_pointer->torque[j]=0;
                                std::cout << "Warning: Knee joint limit reached" << std::endl;
                              }
                            break;
                            case 1:
                              if ((data_pointer->pos[j]*actuator_conversion_factor > HIPMAX && data_pointer->torque[j] > 0) | (data_pointer->pos[j]*actuator_conversion_factor < HIPMIN && data_pointer->torque[j] < 0)) {
                                data_pointer->torque[j]=0;
                                std::cout << "Warning: Hip joint limit reached" << std::endl;
                              }
                            break;
                            case 2:
                              if ((data_pointer->pos[j]*actuator_conversion_factor > HIPMAX && data_pointer->torque[j] > 0) | (data_pointer->pos[j]*actuator_conversion_factor < HIPMIN && data_pointer->torque[j] < 0)) {
                                data_pointer->torque[j]=0;
                                std::cout << "Warning: Hip joint limit reached" << std::endl;
                              }
                          }

                          if((val[j+1]->status & 0x0fff) == 0x0237 && reachedInitial[j+1]){
                            target[j+1]->torque = (int16) data_pointer->torque[j];
                          }
                        }
                        // printf("  Time: %" PRId64 "\n",ec_DCtime);
                        // printf("\r");
                        needlf = TRUE;
                    }
                    usleep(1000);
                    // usleep(750);
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

// **************************************************************************************************************************//

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
                     progState = Exit;
                  }
                  else if(ec_slave[slave].state == EC_STATE_SAFE_OP)
                  {
                    printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                    ec_slave[slave].state = EC_STATE_OPERATIONAL;
                    ec_writestate(slave);
                    progState = Exit;
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
                        progState = Exit;
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
        usleep(250);
    }
}
