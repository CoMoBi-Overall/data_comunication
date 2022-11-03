/**************************************************
---------------- Pre processing ------------------
**************************************************/
/* ---- Basic setting ---- */ 
#include <stdio.h>
#include <cmath>
#include <iostream>
#include "ros/ros.h"
/* ---- Basic setting of Eigen start ---- */ 
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
/* ---- Basic setting of Xenomai start ---- */
#include <alchemy/task.h>
#include <errno.h>
#include <sys/mman.h>
#include <signal.h>
/* ---- Basic setting of EtherCAT start ---- */ 
#include "Can2EtherCAT_Library.h"
#include "MD400T.h"
/* ---- Macro setting ---- */ 
#define dt 0.001
#define ms(x) (x*1000000)

/**************************************************
---------------- Global Variable ------------------
**************************************************/
/* ---- Global variable of Xenomai start ---- */ 
RT_TASK Read_Can_Data_task;
RT_TASK Write_Can_Data_task;
static int run =1;
/* ---- Global variable of EtherCAT start ---- */ 
RT_TASK EtherCAT_task; // Essential for EtherCAT
RT_TASK ecat_ch; // Essential for EtherCAT
RT_TASK print_task; // Print task
RT_TASK Navi_Pub_task; // Navigation Stack Publish task

int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;
EthercatMaster ethercatMaster;

/*Command Data*/
uint8_t read_PID = 1;
uint8_t write_PID = PID_PNT_VEL_CMD;
uint8_t req_data = 0xC4;
uint8_t cmd_vel_L = 60;
uint8_t cmd_vel_R = 0;

/*Motor Driver Value, encoder & wheel velocity*/
uint8_t count_L = 0;
uint8_t count_R = 0;
uint8_t rpm_L = 0;
uint8_t rpm_R = 0;
/* ---- Global variable of Anybus start ---- */

// Transmit part
#define NUM_HMS_ANYBUS 1
uint8_t     OutputByte0000[NUM_HMS_ANYBUS] = {0,}; //CAN DATA 1
uint8_t     OutputByte0001[NUM_HMS_ANYBUS] = {0,}; //CAN DATA 2
uint8_t     OutputByte0002[NUM_HMS_ANYBUS] = {0,}; //CAN DATA 3
uint8_t     OutputByte0003[NUM_HMS_ANYBUS] = {0,}; //CAN DATA 4
uint8_t     OutputByte0004[NUM_HMS_ANYBUS] = {0,}; //CAN DATA 5
uint8_t     OutputByte0005[NUM_HMS_ANYBUS] = {0,}; //CAN DATA 6
uint8_t     OutputByte0006[NUM_HMS_ANYBUS] = {0,}; //CAN DATA 7
uint8_t     OutputByte0007[NUM_HMS_ANYBUS] = {0,}; //CAN DATA 8

// Receive part
uint8_t     InputByte0000[NUM_HMS_ANYBUS] = {0,};
uint8_t     InputByte0001[NUM_HMS_ANYBUS] = {0,};
uint8_t     InputByte0002[NUM_HMS_ANYBUS] = {0,};
uint8_t     InputByte0003[NUM_HMS_ANYBUS] = {0,};
uint8_t     InputByte0004[NUM_HMS_ANYBUS] = {0,}; // Data 1
uint8_t     InputByte0005[NUM_HMS_ANYBUS] = {0,};
uint8_t     InputByte0006[NUM_HMS_ANYBUS] = {0,};
uint8_t     InputByte0007[NUM_HMS_ANYBUS] = {0,};
uint8_t     InputByte0008[NUM_HMS_ANYBUS] = {0,};
uint8_t     InputByte0009[NUM_HMS_ANYBUS] = {0,};
uint8_t     InputByte0010[NUM_HMS_ANYBUS] = {0,};
uint8_t     InputByte0011[NUM_HMS_ANYBUS] = {0,}; 
uint8_t     InputByte0012[NUM_HMS_ANYBUS] = {0,}; // Data 2
uint8_t     InputByte0013[NUM_HMS_ANYBUS] = {0,};
uint8_t     InputByte0014[NUM_HMS_ANYBUS] = {0,};
uint8_t     InputByte0015[NUM_HMS_ANYBUS] = {0,};
uint8_t     InputByte0016[NUM_HMS_ANYBUS] = {0,};
uint8_t     InputByte0017[NUM_HMS_ANYBUS] = {0,};
uint8_t     InputByte0018[NUM_HMS_ANYBUS] = {0,};
uint8_t     InputByte0019[NUM_HMS_ANYBUS] = {0,};
uint8_t     InputByte0020[NUM_HMS_ANYBUS] = {0,}; // Data 3
uint8_t     InputByte0021[NUM_HMS_ANYBUS] = {0,};
uint8_t     InputByte0022[NUM_HMS_ANYBUS] = {0,};
uint8_t     InputByte0023[NUM_HMS_ANYBUS] = {0,};
uint8_t     InputByte0024[NUM_HMS_ANYBUS] = {0,};
uint8_t     InputByte0025[NUM_HMS_ANYBUS] = {0,};
uint8_t     InputByte0026[NUM_HMS_ANYBUS] = {0,};
uint8_t     InputByte0027[NUM_HMS_ANYBUS] = {0,};

/**************************************************
--------------- Xenomai functions -----------------
**************************************************/

uint8_t monitor_can0;

uint64_t test_counter = 0;


void EtherCAT(void *arg) // For transmit & receive by EtherCAT protocol
{
   
	RTIME now, previous;
	rt_task_set_periodic(NULL, TM_NOW, (int)ms(dt*1000*20));
	// RosNodeClass RosNodepub;
	ROS_INFO("hello_EtherCAT");

	previous = rt_timer_read();
    while(run)
    {
		rt_task_wait_period(NULL);
    	now = rt_timer_read();
		if (!run)
		{
			break;
		}
		wkc = ethercatMaster.processTxDomain(); // read the data from anybus
      // from now on, the data start
		ethercatMaster.readBuffer(INPUT_BYTE_0000, InputByte0000);
		ethercatMaster.readBuffer(INPUT_BYTE_0001, InputByte0001);
		ethercatMaster.readBuffer(INPUT_BYTE_0002, InputByte0002);
		ethercatMaster.readBuffer(INPUT_BYTE_0003, InputByte0003);
		ethercatMaster.readBuffer(INPUT_BYTE_0004, InputByte0004);
		ethercatMaster.readBuffer(INPUT_BYTE_0005, InputByte0005);
		ethercatMaster.readBuffer(INPUT_BYTE_0006, InputByte0006);
		ethercatMaster.readBuffer(INPUT_BYTE_0007, InputByte0007);
		// ethercatMaster.readBuffer(INPUT_BYTE_0008, InputByte0008);
		// ethercatMaster.readBuffer(INPUT_BYTE_0009, InputByte0009);
		// ethercatMaster.readBuffer(INPUT_BYTE_0010, InputByte0010);
		// ethercatMaster.readBuffer(INPUT_BYTE_0011, InputByte0011);

		if(wkc >= expectedWKC) 
		{
			// temperatureControl();
			// current_control(0);
		}
		needlf = TRUE;

      #if 1

      OutputByte0000[0] = 0x04;
      OutputByte0001[0] = 0xc4;

      ethercatMaster.writeBuffer(OUTPUT_BYTE_0000, OutputByte0000);
      ethercatMaster.writeBuffer(OUTPUT_BYTE_0001, OutputByte0001);
      ethercatMaster.writeBuffer(OUTPUT_BYTE_0002, OutputByte0002);
      ethercatMaster.writeBuffer(OUTPUT_BYTE_0003, OutputByte0003);
      ethercatMaster.writeBuffer(OUTPUT_BYTE_0004, OutputByte0004);
      ethercatMaster.writeBuffer(OUTPUT_BYTE_0005, OutputByte0005);
      ethercatMaster.writeBuffer(OUTPUT_BYTE_0006, OutputByte0006);
      ethercatMaster.writeBuffer(OUTPUT_BYTE_0007, OutputByte0007);
      #endif 

      ethercatMaster.processRxDomain(); // transmit the data to anybus

      test_counter ++;
		previous = now;
	}

}

void ecatcheck( void *ptr ) // For check the EtherCAT status
{
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
        usleep(250);
    }
}


void signal_handler(int sig)
{
   rt_task_delete(&Read_Can_Data_task);
   rt_task_delete(&Write_Can_Data_task); 
	rt_task_delete(&EtherCAT_task);
   rt_task_delete(&print_task);
	rt_task_delete(&ecat_ch);
   rt_task_delete(&Navi_Pub_task);

   run = 0;
   exit(true);
}

void print(void *arg)
{
    RTIME now, previous;
    rt_task_set_periodic(NULL, TM_NOW, (int)ms(dt*1000));
    ROS_INFO("hello_printing");

    previous = rt_timer_read();   
    while(run)
    {
        rt_task_wait_period(NULL);
        now = rt_timer_read();
        if (!run)
        {
            break;
        }
        previous = now;
   
      rt_printf("Data : %d, %d, %d, %d, %d, %d, %d, %d wkc : %d \n",InputByte0000[0],InputByte0001[0],InputByte0002[0],InputByte0003[0],InputByte0004[0],InputByte0005[0],InputByte0006[0],InputByte0007[0], wkc);

    }

}

void Navi_Pub(void *arg)
{
    RTIME now, previous;
    rt_task_set_periodic(NULL, TM_NOW, (int)ms(dt*1000));
    ROS_INFO("navigation_publishing");


    previous = rt_timer_read();   
    while(run)
    {
        rt_task_wait_period(NULL);
        now = rt_timer_read();
        if (!run)
        {
            break;
        }
        previous = now;

    }

}

void Write_Can_Data(void *arg)
{
   RTIME now, previous;
   rt_task_set_periodic(NULL, TM_NOW, (int)ms(dt*1000*20));
   ROS_INFO("hello_main");

   previous = rt_timer_read();   
   while(run)
   {
      rt_task_wait_period(NULL);
      now = rt_timer_read();
      if (!run)
      {
         break;
      }
      previous = now;
      ////////////////////////// code start //////////////////////
      switch(write_PID)
      {
         case PID_REQ_PID_DATA:
         {
            OutputByte0000[0] = PID_REQ_PID_DATA;
            OutputByte0001[0] = req_data;
            OutputByte0002[0] = 0;
            OutputByte0003[0] = 0;
            OutputByte0004[0] = 0;
            OutputByte0006[0] = 0;
            OutputByte0007[0] = 0;
         }
         case PID_PNT_VEL_CMD:
         {
            OutputByte0000[0] = PID_PNT_VEL_CMD;
            OutputByte0001[0] = 1;
            OutputByte0002[0] = ((cmd_vel_L>>8) & 0Xff);
            OutputByte0003[0] = (cmd_vel_L & 0Xff);

            OutputByte0004[0] = 1;
            OutputByte0005[0] = ((cmd_vel_R>>8) & 0Xff);
            OutputByte0006[0] = (cmd_vel_R & 0Xff);
            OutputByte0007[0] = 0x02;
         }
         case PID_TQ_OFF:
         {
            OutputByte0000[0] = PID_TQ_OFF;
            OutputByte0001[0] = 0;
            OutputByte0002[0] = 0;
            OutputByte0003[0] = 0;
            OutputByte0004[0] = 0;
            OutputByte0005[0] = 0;
            OutputByte0006[0] = 0;
            OutputByte0007[0] = 0;
         }
      }
      
   }
}

void Read_Can_Data(void *arg)
{
   RTIME now, previous;
   rt_task_set_periodic(NULL, TM_NOW, (int)ms(dt*1000*20));
   ROS_INFO("hello_main");

   previous = rt_timer_read();   
   while(run)
   {
      rt_task_wait_period(NULL);
      now = rt_timer_read();
      if (!run)
      {
         break;
      }
      previous = now;
      ////////////////////////// code start //////////////////////
      read_PID = InputByte0000[0];    
      switch(read_PID)
      {
         case PID_MONITOR: // 196 motor L
         {
            // rpm_L = (InputByte0003[0] | InputByte0002[0]<<8);
            rpm_L = BYTE2Int(InputByte0003[0],  InputByte0002[0]);
            // count_L  = (InputByte0007[0] | InputByte0006[0]<<8 | InputByte0005[0]<<16 | InputByte0004[0]<<24);                        
            count_L  = BYTE2LongInt(InputByte0007[0], InputByte0006[0], InputByte0005[0], InputByte0004[0]);
         }

         case PID_MONITOR2: // 201 motor R
         {
            // rpm_R = (InputByte0003[0] | InputByte0002[0]<<8);
            rpm_R = BYTE2Int(InputByte0003[0],  InputByte0002[0]);
            // count_R  = (InputByte0007[0] | InputByte0006[0]<<8 | InputByte0005[0]<<16 | InputByte0004[0]<<24);                        
            count_R  = BYTE2LongInt(InputByte0007[0], InputByte0006[0], InputByte0005[0], InputByte0004[0]);
         }
      }
      
   }
}
/**************************************************
--------------------- Xenomai Main ----------------
**************************************************/

int main(int argc, char **argv)
{
   ros::init(argc,argv,"Anybus_test");
	printf("SOEM (Simple Open EtherCAT Master)\nSimple test\n");

	rt_task_create(&ecat_ch, "EtherCAT_checking", 0, 99, 0);
	rt_task_start(&ecat_ch, &ecatcheck, NULL);

   signal(SIGTERM, signal_handler); //Termination
   signal(SIGINT, signal_handler); //Active

   mlockall(MCL_CURRENT|MCL_FUTURE);
    
	inOP = ethercatMaster.init(TORQUE_MODE, "enp2s0");
	
	if(inOP == FALSE)
	{
		printf("System Initialization Failed\n");
        return 0;
	}

   rt_task_create(&Write_Can_Data_task, "Write_Can_Data", 0, 99, 0);
   rt_task_start(&Write_Can_Data_task, &Write_Can_Data, NULL);
   rt_task_create(&Read_Can_Data_task, "Read_Can_Data", 0, 99, 0); // 99 is priority. 99 is max. priority
   rt_task_start(&Read_Can_Data_task, &Read_Can_Data, NULL);
   rt_task_create(&EtherCAT_task, "EtherCAT_tasking", 0, 99, 0);
   rt_task_start(&EtherCAT_task, &EtherCAT, NULL);
   rt_task_create(&print_task, "printing", 0, 99, 0);
   rt_task_start(&print_task, &print, NULL);
   rt_task_create(&Navi_Pub_task, "Navi_Pub_task", 0, 99, 0);
   rt_task_start(&Navi_Pub_task, &Navi_Pub, NULL);

   pause();

   rt_task_delete(&Read_Can_Data_task); // task 닫을때 필요함
   rt_task_delete(&Write_Can_Data_task); 
	rt_task_delete(&EtherCAT_task);
   rt_task_delete(&print_task);
	rt_task_delete(&ecat_ch);
   rt_task_delete(&Navi_Pub_task);

   return 0;
}


