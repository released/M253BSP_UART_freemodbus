# M253BSP_UART_freemodbus
 M253BSP_UART_freemodbus

update @ 2023/01/31

1. Two KEIL project , by different define : ENABLE_MODBUS_MASTER , ENABLE_MODBUS_SLAVE

2. use UART0 : PB.12(RX) , PB.13 (TX) for Freemodbus communication

refer below link for reference , to porting Freemodbus library , check folder : port , to modify UART/TIMER function , base on MCU P/N

https://github.com/OpenNuvoton/NUC230_240BSP/tree/master/SampleCode/NuEdu/Smpl_Basic01_Modbus_UART

http://www.nuvoton-mcu.com/forum.php?mod=viewthread&tid=1686

https://blog.csdn.net/qq_42992084/article/details/107590803

https://www.amobbs.com/thread-5491615-1-1.html

https://github.com/RT-Thread-packages/freemodbus

https://github.com/armink/FreeModbus_Slave-Master-RTT-STM32

3. when use PROJECT define : ENABLE_MODBUS_MASTER , below is MCU log , when receive slave device data

![image](https://github.com/released/M253BSP_UART_freemodbus/blob/main/master_log.jpg)	

4. below is LA UART TX , RX capture ( 2 MCU board , one act as master , one act as slave device )

MASTER send data on TX , to slave device ( device ID : 10 , length : 100 , addr : 1000 , 04:INPUT REGISTER )

![image](https://github.com/released/M253BSP_UART_freemodbus/blob/main/input_register_LA_master.jpg)

MASTER receive data on RX , from slave device response , first bytes use 0xA5A5 , last byte use 0x5A5A for indicator
		
![image](https://github.com/released/M253BSP_UART_freemodbus/blob/main/input_register_LA_slv_startjpg)	

![image](https://github.com/released/M253BSP_UART_freemodbus/blob/main/input_register_LA_slv_end.jpg)	

5. under folder : test_tool , is the tool , to test modbus MASTER or SLAVE function

need a UART bridge for PC , to communcation with MCU modbus

PC_as_Slave_MCU_as_Master 

use diagslave.bat , to test master function

![image](https://github.com/released/M253BSP_UART_freemodbus/blob/main/tool_diagslave.jpg)	


PC_as_Master_MCU_as_Slave :  

a.ModScan32\ModScan32.exe , to test slave function

![image](https://github.com/released/M253BSP_UART_freemodbus/blob/main/ModScan32.jpg)	

b.modpoll_input_register.bat , to test slave function 

![image](https://github.com/released/M253BSP_UART_freemodbus/blob/main/tool_modpoll_log1.jpg)	

