#include "Can2EtherCAT_Library.h"
#include <iostream>

using namespace std;

EthercatMaster::EthercatMaster(){}
EthercatMaster::~EthercatMaster()
{
    ec_slave[0].state = EC_STATE_INIT;
	/* request INIT state for all slaves */
	ec_writestate(0);


	printf("End simple test, close socket\n");
	/* stop SOEM, close socket */
	ec_close();
	
}

int EthercatMaster::init(const int8_t mode_op, char *ifname)
{      
   //Ethercat SOEM
	// char *ifname;
	// ifname = "enp2s0";
	int j, oloop, iloop, wkc_count, chk;

    //inOP = FALSE;

	 if (ec_init(ifname))
    {
        printf("ec_init on %s succeeded.\n",ifname);
        /* find and auto-config slaves */

        /** network discovery */
        if ( ec_config_init(FALSE) > 0 )
        {
            printf("%d slaves found and configured.\n", ec_slavecount);

			//Elmo Config
			this->initSlave();
            
            /** if CA disable => automapping works */
            ec_config_map(&IOmap);
			
            // show slave info
			this->showSlaveInfo();
			
            printf("Slaves mapped, state to SAFE_OP.\n");

            int timestep = 700;

            /* wait for all slaves to reach SAFE_OP state */
            printf("test00\n");
			ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);
			
			/** old SOEM code, inactive */
            oloop = ec_slave[0].Obytes;
            if ((oloop == 0) && (ec_slave[0].Obits > 0)) oloop = 1;
            if (oloop > 20) oloop = 8;
            iloop = ec_slave[0].Ibytes;
            if ((iloop == 0) && (ec_slave[0].Ibits > 0)) iloop = 1;
            if (iloop > 20) iloop = 8;

			//printf("test11\n");
            int expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            printf("Calculated workcounter %d\n", expectedWKC);

			//Elmo output initialize
			
			

            /** going operational */
            ec_slave[0].state = EC_STATE_OPERATIONAL;
            /* send one valid process data to make outputs in slaves happy*/
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            
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

			this->registerParam();
			
            if (ec_slave[0].state == EC_STATE_OPERATIONAL )
            {

				printf("Operational state reached for all slaves.\n");
                wkc_count = 0;
                //inOP = TRUE;

                /**
                 * Drive state machine transistions
                 *   0 -> 6 -> 7 -> 15
                 */
                this->testDriveState(mode_op);
				
				return 1;
            }
            else
            {
                printf("Not all slaves reached operational state.\n");
                ec_readstate();
                for(int i = 1; i<=ec_slavecount ; i++)
                {
                    if(ec_slave[i].state != EC_STATE_OPERATIONAL)
                    {
                        printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                            i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                    }
                }
            }
        }
        else
        {
            printf("No slaves found!\n");
        }
    }
    else
    {
        printf("No socket connection on %s\nExcecute as root\n",ifname);
    }

	return 0;
}
                                                                                                                                                                                            
int Elmosetup(uint16 slave) // must be set up(by yoon)
{
    int retval;
    uint16 u16val;
    
    retval = 0;
	
	uint16 map_1c12[] = {0x0002, 0x1605, 0x161D};
	uint16 map_1c13[] = {0x000B,  0x1A0E, 0x1A13, 0x1A0A, 0x1A0B, 0x1A0B, 0x1A11, 0x1A18, 0x1A1C, 0x1A1E,  0x1A1D, 0x1A1F};
	//uint16 map_1c13[] = {0x0007,  0x1A02,  0x1A11, 0x1A18, 0x1A1C,  0x1A1E, 0x1A1D, 0x1A1F};

	retval +=ec_SDOwrite(slave,0x1c12,0x00,TRUE,sizeof(map_1c12),&map_1c12,EC_TIMEOUTSAFE);
	retval +=ec_SDOwrite(slave,0x1c13,0x00,TRUE,sizeof(map_1c13),&map_1c13,EC_TIMEOUTSAFE);
    printf("EL7031 slave %d set, retval = %d\n", slave, retval);
    return 1;
}

int AnybusSetup(uint16 slave) // must be set up(by yoon)
{
    int retval;                              
    uint16 u16val;
    
    retval = 0;
	
	//uint16 map_1c12[] = {0x0002, 0x1605, 0x161D};
	// uint16 map_1c13[] = {(uint16)0x20001,  (uint16)0x20002, (uint16)0x20003, (uint16)0x20004, 
	//                      (uint16)0x20005, (uint16)0x20006, (uint16)0x20007, (uint16)0x20008, (uint16)0x20009, (uint16)0x20010,  (uint16)0x200011};
	uint16 map_1c13[] = {(uint16)0x20001,  (uint16)0x20002, (uint16)0x20003, (uint16)0x20004, 
	                     (uint16)0x20005, (uint16)0x20006}; // two byte is allocated
	//uint16 map_1c13[] = {0x0007,  0x1A02,  0x1A11, 0x1A18, 0x1A1C,  0x1A1E, 0x1A1D, 0x1A1F};

	//retval +=ec_SDOwrite(slave,0x1c12,0x00,TRUE,sizeof(map_1c12),&map_1c12,EC_TIMEOUTSAFE);
	retval +=ec_SDOwrite(slave,0x1c13,0x00,TRUE,sizeof(map_1c13),&map_1c13,EC_TIMEOUTSAFE);
    printf("EL7031 slave %d set, retval = %d\n", slave, retval);
    return 1;
}

int EthercatMaster::initSlave() // must be set up(by yoon)
{
	for(int i = 1 ; i <= NUM_ACTUATOR ; i++)
	{
		if((ec_slave[i].eep_man == ELMO_VENDOR_ID) && (ec_slave[i].eep_id == ELMO_GOLD_PRODUCT_CODE))
		{
			printf("Found %s at position %d\n", ec_slave[i].name, i);
			/* link slave specific setup to preop->safeop hook */
			ec_slave[i].PO2SOconfig = Elmosetup;
			//Elmosetup(i);
			// int retval;
			// uint16_t u16val;
			
			// retval = 0;
			
			// uint16_t map_1c12[] = {0x0002, 0x1605, 0x161D};
			// uint16_t map_1c13[] = {0x000B,  0x1A0E, 0x1A13, 0x1A0A, 0x1A0B, 0x1A0B, 0x1A11, 0x1A18, 0x1A1C, 0x1A1E,  0x1A1D, 0x1A1F};
			// //uint16 map_1c13[] = {0x0007,  0x1A02,  0x1A11, 0x1A18, 0x1A1C,  0x1A1E, 0x1A1D, 0x1A1F};

			// retval +=ec_SDOwrite(i,0x1c12,0x00,TRUE,sizeof(map_1c12),&map_1c12,EC_TIMEOUTSAFE);
			// retval +=ec_SDOwrite(i,0x1c13,0x00,TRUE,sizeof(map_1c13),&map_1c13,EC_TIMEOUTSAFE);
			// printf("EL7031 slave %d set, retval = %d\n", i, retval);
		}
	}

	for (int i = NUM_ACTUATOR + 1; i<= NUM_ACTUATOR + NUM_HMS_ANYBUS; i++)
	{
		if((ec_slave[i].eep_man == HMS_VENDOR_ID) && (ec_slave[i].eep_id == HMS_ANYBUS_PRODUCT_CODE))
		{
			printf("Found %s at position %d\n", ec_slave[i].name, i);
			/* link slave specific setup to preop->safeop hook */
			ec_slave[i].PO2SOconfig = AnybusSetup;
		}
	}


	return 0;
}

int EthercatMaster::showSlaveInfo()
{
	for (int i=1; i<=ec_slavecount; i++) 
	{
		printf("\nSlave:%d\n Name:%s\n Output size: %dbits\n Input size: %dbits\n State: %d\n Delay: %d[ns]\n Has DC: %d\n",
		i, ec_slave[i].name, ec_slave[i].Obits, ec_slave[i].Ibits,
		ec_slave[i].state, ec_slave[i].pdelay, ec_slave[i].hasdc);
	}  
}
int EthercatMaster::registerParam()
{
	
	for(int i = 0 ; i < NUM_ACTUATOR ; i++)
	{
		_elmo_gold[i].InParam = (struct ELMO_GOLD_IN *)(ec_slave[i + 1].outputs);
		_elmo_gold[i].OutParam = (struct ELMO_GOLD_OUT *)(ec_slave[i + 1].inputs);
	}

	for(int i = NUM_ACTUATOR ; i < NUM_ACTUATOR + NUM_HMS_ANYBUS ; i++)
	{
		_hms_anybus[i - NUM_ACTUATOR].InParam = (struct HMS_ANYBUS_IN *)(ec_slave[i + 1].outputs); // transmit to anybus
		_hms_anybus[i - NUM_ACTUATOR].OutParam = (struct HMS_ANYBUS_OUT *)(ec_slave[i + 1].inputs); // get from the anybus 
	}

}
int EthercatMaster::testDriveState(const int8_t mode_op)
{
	for (int i=0; i<NUM_ACTUATOR; i++) 
	{

		_elmo_gold[i].InParam->ModeOfOperation = mode_op;
		_elmo_gold[i].InParam->MaxTorque = 1000;
		ec_send_processdata();
		
		ec_receive_processdata(EC_TIMEOUTRET); 
		_elmo_gold[i].InParam->ControlWord = 0x6;
		ec_send_processdata();
		ec_receive_processdata(EC_TIMEOUTRET);  
		_elmo_gold[i].InParam->ControlWord = 0x7;
		ec_send_processdata();
		ec_receive_processdata(EC_TIMEOUTRET); 

		_elmo_gold[i].InParam->ControlWord = 0x0F;
		ec_send_processdata();
		ec_receive_processdata(EC_TIMEOUTRET); 

	}
}

void EthercatMaster::processRxDomain() // 한번에 다 받는 함수
{
	ec_send_processdata();	
}

int EthercatMaster::processTxDomain() // 한번에 다 보내는 함수
{
	return ec_receive_processdata(EC_TIMEOUTRET);		
}


void EthercatMaster::writeBuffer(const int EntryID, void* data) //void* Generic pointer:모든 데이터의 자료형의 데이터를 가리킬 수 있는 포인터
{
    switch (EntryID)
	{
		// Anybus write: 1

		case OUTPUT_BYTE_0000:
		{
			uint8_t* _OutputByte0000= static_cast<uint8_t *>(data);
            for (int i=0; i<NUM_HMS_ANYBUS; i++)
            {
                _hms_anybus[i].InParam->OutputByte0000= _OutputByte0000[i];
			}
		}
			break;
		case OUTPUT_BYTE_0001:
		{
			uint8_t* _OutputByte0001= static_cast<uint8_t *>(data);
            for (int i=0; i<NUM_HMS_ANYBUS; i++)
            {
                _hms_anybus[i].InParam->OutputByte0001= _OutputByte0001[i];
			}
		}
			break;
		case OUTPUT_BYTE_0002:
		{
			uint8_t* _OutputByte0002= static_cast<uint8_t *>(data);
            for (int i=0; i<NUM_HMS_ANYBUS; i++)
            {
                _hms_anybus[i].InParam->OutputByte0002= _OutputByte0002[i];
			}
		}
			break;
		case OUTPUT_BYTE_0003:
		{
			uint8_t* _OutputByte0003= static_cast<uint8_t *>(data);
            for (int i=0; i<NUM_HMS_ANYBUS; i++)
            {
                _hms_anybus[i].InParam->OutputByte0003= _OutputByte0003[i];
			}
		}
			break;
		case OUTPUT_BYTE_0004:
		{
			uint8_t* _OutputByte0004= static_cast<uint8_t *>(data);
            for (int i=0; i<NUM_HMS_ANYBUS; i++)
            {
                _hms_anybus[i].InParam->OutputByte0004= _OutputByte0004[i];
			}
		}
			break;

		case OUTPUT_BYTE_0005:
		{
			uint8_t* _OutputByte0005= static_cast<uint8_t *>(data);
            for (int i=0; i<NUM_HMS_ANYBUS; i++)
            {
                _hms_anybus[i].InParam->OutputByte0005= _OutputByte0005[i];
			}
		}
			break;

		case OUTPUT_BYTE_0006:
		{
			uint8_t* _OutputByte0006= static_cast<uint8_t *>(data);
            for (int i=0; i<NUM_HMS_ANYBUS; i++)
            {
                _hms_anybus[i].InParam->OutputByte0006= _OutputByte0006[i];
			}
		}
			break;
		case OUTPUT_BYTE_0007:
		{
			uint8_t* _OutputByte0007= static_cast<uint8_t *>(data);
            for (int i=0; i<NUM_HMS_ANYBUS; i++)
            {
                _hms_anybus[i].InParam->OutputByte0007= _OutputByte0007[i];
			}
		}
			break;
							
		default:	// Undefined Entry ID	
			
			break;
	}
}

void EthercatMaster::readBuffer(const int EntryID, void* data)
{
    switch (EntryID)
	{		
		case INPUT_BYTE_0000:
		{
            uint8_t * _InputByte0000 = static_cast<uint8_t * const>(data);
            for (int i=0; i<NUM_HMS_ANYBUS; i++)
            {
                _InputByte0000[i] = _hms_anybus[i].OutParam->InputByte0000;
            }
			
		}
			break;		

		case INPUT_BYTE_0001:
		{
            uint8_t * _InputByte0001 = static_cast<uint8_t * const>(data);
            for (int i=0; i<NUM_HMS_ANYBUS; i++)
            {
                _InputByte0001[i] = _hms_anybus[i].OutParam->InputByte0001;
            }
			
		}
			break;		

		case INPUT_BYTE_0002:
		{
            uint8_t * _InputByte0002 = static_cast<uint8_t * const>(data);
            for (int i=0; i<NUM_HMS_ANYBUS; i++)
            {
                _InputByte0002[i] = _hms_anybus[i].OutParam->InputByte0002;
            }
		}
			break;			
		case INPUT_BYTE_0003:
		{
            uint8_t * _InputByte0003 = static_cast<uint8_t * const>(data);
            for (int i=0; i<NUM_HMS_ANYBUS; i++)
            {
                _InputByte0003[i] = _hms_anybus[i].OutParam->InputByte0003;
            }
		}
			break;	
		case INPUT_BYTE_0004:
		{
            uint8_t * _InputByte0004 = static_cast<uint8_t * const>(data);
            for (int i=0; i<NUM_HMS_ANYBUS; i++)
            {
                _InputByte0004[i] = _hms_anybus[i].OutParam->InputByte0004;
            }
		}
			break;	
		case INPUT_BYTE_0005:
		{
            uint8_t * _InputByte0005 = static_cast<uint8_t * const>(data);
            for (int i=0; i<NUM_HMS_ANYBUS; i++)
            {
                _InputByte0005[i] = _hms_anybus[i].OutParam->InputByte0005;
            }
		}
			break;	
		case INPUT_BYTE_0006:
		{
            uint8_t * _InputByte0006 = static_cast<uint8_t * const>(data);
            for (int i=0; i<NUM_HMS_ANYBUS; i++)
            {
                _InputByte0006[i] = _hms_anybus[i].OutParam->InputByte0006;
            }
		}
			break;	
		case INPUT_BYTE_0007:
		{
            uint8_t * _InputByte0007 = static_cast<uint8_t * const>(data);
            for (int i=0; i<NUM_HMS_ANYBUS; i++)
            {
                _InputByte0007[i] = _hms_anybus[i].OutParam->InputByte0007;
            }
		}
			break;
		
		#if 0
		case INPUT_BYTE_0008:
		{
            uint8_t * _InputByte0008 = static_cast<uint8_t * const>(data);
            for (int i=0; i<NUM_HMS_ANYBUS; i++)
            {
                _InputByte0008[i] = _hms_anybus[i].OutParam->InputByte0008;
            }
		}
			break;
		case INPUT_BYTE_0009:
		{
            uint8_t * _InputByte0009 = static_cast<uint8_t * const>(data);
            for (int i=0; i<NUM_HMS_ANYBUS; i++)
            {
                _InputByte0009[i] = _hms_anybus[i].OutParam->InputByte0009;
            }
		}
			break;
		case INPUT_BYTE_0010:
		{
            uint8_t * _InputByte0010 = static_cast<uint8_t * const>(data);
            for (int i=0; i<NUM_HMS_ANYBUS; i++)
            {
                _InputByte0010[i] = _hms_anybus[i].OutParam->InputByte0010;
            }
		}
			break;
		case INPUT_BYTE_0011:
		{
            uint8_t * _InputByte0011 = static_cast<uint8_t * const>(data);
            for (int i=0; i<NUM_HMS_ANYBUS; i++)
            {
                _InputByte0011[i] = _hms_anybus[i].OutParam->InputByte0011;
            }
		}
			break;

		#endif

		default:	// Undefined Entry ID
		//std::cout<< "error!!!~~"<< std::hex << EntryID << INPUT_BYTE_0027 << "\t";
			break;
	}
}
