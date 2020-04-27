/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <logging/log.h>
#include <errno.h>
#include <sys/printk.h>
#include <sys/byteorder.h>
#include <device.h>
#include <string.h>
#include <net/socket.h>
#include <net/net_mgmt.h>
#include <net/udp.h>
#include <net/coap.h>
#include <net/net_ip.h>

#include <sensor.h>
#include <i2c.h>
#include <gpio.h>
#include <uart.h>
//  #include <../subsys/net/ip/net_private.h>

#define PEER_PORT 5683
#define MAX_COAP_MSG_LEN 256
#define PORT   "GPIOA"
#define PIN    4
#define PIN5    5

#define LED_PORT3	DT_GPIO_LEDS_LED_3_GPIOS_CONTROLLER
#define LED_PORT4	DT_GPIO_LEDS_LED_4_GPIOS_CONTROLLER
#define LED_PORT5	DT_GPIO_LEDS_LED_5_GPIOS_CONTROLLER
#define LED_PORT6	DT_GPIO_LEDS_LED_6_GPIOS_CONTROLLER
#define LED_PORT7	DT_GPIO_LEDS_LED_7_GPIOS_CONTROLLER
#define LED_PORT8	DT_GPIO_LEDS_LED_8_GPIOS_CONTROLLER
#define LED_PORT9	DT_GPIO_LEDS_LED_9_GPIOS_CONTROLLER
#define LED_PORT10	DT_GPIO_LEDS_LED_10_GPIOS_CONTROLLER
#define LED3 DT_GPIO_LEDS_LED_3_GPIOS_PIN
#define LED4 DT_GPIO_LEDS_LED_4_GPIOS_PIN
#define LED5 DT_GPIO_LEDS_LED_5_GPIOS_PIN
#define LED6 DT_GPIO_LEDS_LED_6_GPIOS_PIN
#define LED7 DT_GPIO_LEDS_LED_7_GPIOS_PIN
#define LED8 DT_GPIO_LEDS_LED_8_GPIOS_PIN
#define LED9 DT_GPIO_LEDS_LED_9_GPIOS_PIN
#define LED10 DT_GPIO_LEDS_LED_10_GPIOS_PIN
#define PI                         (float)     3.14159265f
#define ACC DT_INST_0_ST_LSM303DLHC_ACCEL_IRQ_GPIOS_CONTROLLER_0
#define SLEEP_TIME	1000
const uint16_t polynom = 0xA001;
static int sock;
struct pollfd fds[1];
static int nfds;

int floattoint(float a)
{
	return (int)(a+0.5)>=360?(int)(a+0.5)-360:(int)(a+0.5);
}
static void prepare_fds(void)
{
	fds[nfds].fd = sock;
	fds[nfds].events = POLLIN;
	nfds++;
}

static int start_coap_client(void)
{
	int ret = 0;
	struct sockaddr_in6 addr6;

	addr6.sin6_family = AF_INET6;
	addr6.sin6_port = htons(PEER_PORT);
	addr6.sin6_scope_id = 0U;

	inet_pton(AF_INET6, CONFIG_NET_CONFIG_PEER_IPV6_ADDR,
		  &addr6.sin6_addr);

	sock = socket(addr6.sin6_family, SOCK_DGRAM, IPPROTO_UDP);
	if (sock < 0) {
		printf("Failed to create UDP socket %d", errno);
		return -errno;
	}

	ret = connect(sock, (struct sockaddr *)&addr6, sizeof(addr6));
	if (ret < 0) {
		printf("Cannot connect to UDP remote : %d", errno);
		return -errno;
	}

	prepare_fds();

	return 0;
}


static int send_simple_coap_request(u8_t method,char** test_path,u8_t* sensor_data)
{
	struct coap_packet request;
	char **p;
	u8_t *data;
	int r;

	data = (u8_t *)k_malloc(MAX_COAP_MSG_LEN);
	if (!data) {
		return -ENOMEM;
	}

	r = coap_packet_init(&request, data, MAX_COAP_MSG_LEN,
			     1, COAP_TYPE_CON, 8, coap_next_token(),
			     method, coap_next_id());
	if (r < 0) {
		printf("Failed to init CoAP message");
		goto end;
	}

	for (p = test_path; p && *p; p++) {
		r = coap_packet_append_option(&request, COAP_OPTION_URI_PATH,
					      *p, strlen(*p));
		if (r < 0) {
			printf("Unable add option to request");
			goto end;
		}
	}

	switch (method) {
	case COAP_METHOD_GET:
	case COAP_METHOD_DELETE:
		break;

	case COAP_METHOD_PUT:
	case COAP_METHOD_POST:
		r = coap_packet_append_payload_marker(&request);
		if (r < 0) {
			printf("Unable to append payload marker");
			goto end;
		}

		r = coap_packet_append_payload(&request, (u8_t *)sensor_data,
					       30);
        // u8_t  test100[20] = "1 0 0.123456";
		// printf("sizeof(test100) - 1   :%d",sizeof(test100) - 1);
        // r = coap_packet_append_payload(&request, (u8_t *)test100,
		// 			       sizeof(test100) - 1);
		if (r < 0) {
			printf("Not able to append payload");
			goto end;
		}

		break;
	default:
		r = -EINVAL;
		goto end;
	}

	// net_hexdump("Request", request.data, request.offset);

	r = send(sock, request.data, request.offset, 0);

end:
	k_free(data);

	return 0;
}

static int send_simple_coap_msgs_and_wait_for_reply(int sensortype,u8_t* data)
{
    // printf("send_simple_coap:%s  \n",data);
	int r;
	switch (sensortype) {
		case 0://register
		{
			char * register_path[]={"register",NULL};
			r = send_simple_coap_request(COAP_METHOD_POST,register_path,data);
			break;
		}
		case 1:
		{
			char * register_path[]={"1281B_Temp",NULL};
			r = send_simple_coap_request(COAP_METHOD_POST,register_path,data);
			break;
		}
			break;
		case 2:
		{
			/* Test CoAP POST method*/

			char * test_path[]={"1281B_Current",NULL};
			r = send_simple_coap_request(COAP_METHOD_POST,test_path,data);
			if (r < 0) {
				return r;
			}
			break;
		}
		case 5:
		{

			char * test_path[]={"BME_TEMP",NULL};
			r = send_simple_coap_request(COAP_METHOD_POST,test_path,data);
			if (r < 0) {
				return r;
			}
			break;
		}
		case 8:
		{

			char * test_path[]={"rotate",NULL};
			r = send_simple_coap_request(COAP_METHOD_POST,test_path,data);
			if (r < 0) {
				return r;
			}
			break;
		}
		case 4:
		{

			char * test_path[]={"BME_Press",NULL};
			r = send_simple_coap_request(COAP_METHOD_POST,test_path,data);
			if (r < 0) {
				return r;
			}
			break;
		}
		case 7:
		{

			char * test_path[]={"BME_Hum",NULL};
			r = send_simple_coap_request(COAP_METHOD_POST,test_path,data);
			if (r < 0) {
				return r;
			}
			break;
		}

			break;
		default:
			return 0;
		}
	return 0;
}


uint16_t crc16bitbybit(uint8_t *ptr, uint16_t len)
{
	
	uint16_t crc = 0xffff;
	
	uint8_t i;
	
	if (len == 0) {
		len = 1;
	}
	
	while (len--) {
		crc ^= *ptr;
		for (i = 0; i<8; i++)
		{
			if (crc & 1) {
				crc >>= 1;
				crc ^= polynom;
			}
			else {
				crc >>= 1;
			}
		}
		ptr++;
	}
		
	return(crc);
}
static void wait(void)
{
	if (poll(fds, nfds, K_FOREVER) < 0) {
		printf("Error in poll:%d", errno);
	}
}
static int process_simple_coap_reply(void)
{
	struct coap_packet reply;
	u8_t *data;
	int rcvd;
	int ret;

	wait();

	data = (u8_t *)k_malloc(MAX_COAP_MSG_LEN);
	if (!data) {
		return -ENOMEM;
	}

	rcvd = recv(sock, data, MAX_COAP_MSG_LEN, MSG_DONTWAIT);
	if (rcvd == 0) {
		printf("rcvd == 0");
		ret = -1;
		return ret;
	}

	if (rcvd < 0) {
		if (errno == EAGAIN || errno == EWOULDBLOCK) {
			ret = -1;
		} else {
			ret = -1;
		}
		return ret;
	}

	// net_hexdump("Response", data, rcvd);
	// printf("\nrcvd:%d",rcvd);
	// printf("\ndata:%s\n",data);
	ret = coap_packet_parse(&reply, data, rcvd, NULL, 0);
	if (ret < 0) {
		printf("Invalid data received");
		return ret;
	}
	// printf("\nrece data :%s\n",reply.data);
	 u16_t lengthofpayload;
	u8_t* payload;
	payload = coap_packet_get_payload(&reply,&lengthofpayload);
	// printf("\nlength:%d\n",lengthofpayload);
	// printf("payload:%s",payload);

	if(payload[0] == 'y'){
		return 1;
	}else{
		return -1;
	}
}
		u8_t CoapCurrent[30];
		u8_t Coap1281Temp[30];
		u8_t CoapBMETemp[30];
		u8_t Coapxandy[30];
		u8_t CoapBMEPress[30];
		u8_t CoapBMEHum[30];
		unsigned char Rx_Buffer[40];//35 36 check
void main(void)
{

    u32_t ledon = 1;
	u32_t ledoff = 0;
    int ret;

	struct device *devacc;
	// struct device *devmag;
	struct device *led3;
	struct device *led4;
	struct device *led5;
	struct device *led6;
	struct device *led7;
	struct device *led8;
	struct device *led9;
	struct device *led10;
    struct device *uart_dev;
    struct device *bme;

    struct sensor_value sen_valacc[3];
	// struct sensor_value sen_valaccx;
	// struct sensor_value sen_valaccy;
	// struct sensor_value sen_valaccz;
	// struct sensor_value sen_valmag[3];
    struct sensor_value tem;
    struct sensor_value pre;
	struct sensor_value gas;
    struct sensor_value HUM;

    unsigned char send[8] = {0x01,0x03,0x00,0x48,0x00,0x08,0xC4,0x1A};


	led3 = device_get_binding(LED_PORT3);
	led4 = device_get_binding(LED_PORT4);
	led5 = device_get_binding(LED_PORT5);
	led6 = device_get_binding(LED_PORT6);
	led7 = device_get_binding(LED_PORT7);
	led8 = device_get_binding(LED_PORT8);
	led9 = device_get_binding(LED_PORT9);
	led10 = device_get_binding(LED_PORT10);
	gpio_pin_configure(led3, LED3, GPIO_DIR_OUT);
	gpio_pin_configure(led4, LED4, GPIO_DIR_OUT);
	gpio_pin_configure(led5, LED5, GPIO_DIR_OUT);
	gpio_pin_configure(led6, LED6, GPIO_DIR_OUT);
	gpio_pin_configure(led7, LED7, GPIO_DIR_OUT);
	gpio_pin_configure(led8, LED8, GPIO_DIR_OUT);
	gpio_pin_configure(led9, LED9, GPIO_DIR_OUT);
	gpio_pin_configure(led10, LED10, GPIO_DIR_OUT);

for(int i = 0;i<3;i++)
{
	gpio_pin_write(led3, LED3, ledon);
	gpio_pin_write(led4, LED4, ledon);
	gpio_pin_write(led5, LED5, ledon);
	gpio_pin_write(led6, LED6, ledon);
	gpio_pin_write(led7, LED7, ledon);
	gpio_pin_write(led8, LED8, ledon);
	gpio_pin_write(led9, LED9, ledon);
	gpio_pin_write(led10, LED10, ledon);
	k_sleep(500);
	gpio_pin_write(led3, LED3, ledoff);
	gpio_pin_write(led4, LED4, ledoff);
	gpio_pin_write(led5, LED5, ledoff);
	gpio_pin_write(led6, LED6, ledoff);
	gpio_pin_write(led7, LED7, ledoff);
	gpio_pin_write(led8, LED8, ledoff);
	gpio_pin_write(led9, LED9, ledoff);
	gpio_pin_write(led10, LED10, ledoff);
	k_sleep(500);
	
}

    uart_dev = device_get_binding("UART_2");
    bme = device_get_binding("BME680_test");
	devacc = device_get_binding(DT_INST_0_ST_LIS2DH_LABEL);
	if(devacc != NULL)
		printk("device is %p, name is %s\n", devacc, devacc->config->name);
	if(uart_dev != NULL)
		printk("device is %p, name is %s\n", uart_dev, uart_dev->config->name);
	if(bme != NULL)
		printk("device is %p, name is %s\n", bme, bme->config->name);
	// devmag = device_get_binding(DT_INST_0_ST_LSM303DLHC_MAGN_LABEL);
	// printk("device is %p, name is %s\n", devmag, devmag->config->name);
	
    
//////////////////socket udp
	// int sock2;

	// struct sockaddr_in6 addr6;
	// int addr_len = sizeof(struct sockaddr_in6);
	// addr6.sin6_family = AF_INET6;
	// addr6.sin6_port = 8000;
	// addr6.sin6_scope_id = 0U;
	// inet_pton(AF_INET6,  "fe80::204:a3ff:fe10:25", &addr6.sin6_addr);
	// //inet_pton(AF_INET6,  "2001:db5::4cd3:4660:645e:3627", &addr6.sin6_addr);
	// sock2 = socket(addr6.sin6_family, SOCK_DGRAM, IPPROTO_UDP);
	// if (sock2 < 0)
	// {
	// 	printf("\nsock gg sock gg sock gg sock gg\n");
	// }
	// else
	// {
	// 	printf("\nsock OK sock OK sock OK sock OK\n");
	// }
////////////////////////coapintit
    ret = start_coap_client();
    if(ret < 0){
        printf("coap_init failed\n");
    }

//////////////////////register coap post

	
    u8_t register_data[30] = "5 1_0 2_0 5_0 4_0 7_0";
	int register_reply = -1;
	while(register_reply<0){
		send_simple_coap_msgs_and_wait_for_reply(0,register_data);
		register_reply = process_simple_coap_reply();
		// k_sleep(5000);
	}
    
    ///////////////////////////mac
	// struct net_if *iface = net_if_get_default();
	// char *linkaddr = net_sprint_ll_addr(net_if_get_link_addr(iface)->addr,net_if_get_link_addr(iface)->len);
  	// printf("Link addr:%s\n",linkaddr);




        int count = 0;
		float AccBuffer[3];
		float sumaccnony;
        float sumaccnonx;
        float consty,sinconsty,cosconsty;
        float constx,sinconstx,cosconstx;

	
		u16_t after;
	    u8_t hou;
	    u8_t qian;
		unsigned char  recvChar;
        unsigned long Voltage_data,Current_data,Power_data,Energy_data,Pf_data,CO2_data,temperature,HZ;
			

		int num = 0;
        while(1)
        {
            printf("-------------------------------------\n");
//             ///////////////////////////////////////////////devacc && devmag
            sensor_sample_fetch(devacc);
			memset(sen_valacc,0,sizeof(sen_valacc));
            sensor_channel_get(devacc,SENSOR_CHAN_ACCEL_XYZ,sen_valacc);


//             // printf("test point here\n");

//             // sensor_sample_fetch(devmag);
//             // ret = sensor_channel_get(devmag,SENSOR_CHAN_MAGN_XYZ,sen_valmag);
//             // if (ret) 
//             // {
//             //         printk("devmag failed ret %d\n", ret);
//             // }
            
//             // float MagBuffer[3];

			memset(AccBuffer,0,sizeof(AccBuffer));
            for(int i = 0;i<3;i++)
            {
                AccBuffer[i] = ( (float)sen_valacc[i].val1 + 0.000001 * (float)sen_valacc[i].val2 ) ;
                // MagBuffer[i] = ( (float)sen_valmag[i].val1 + 0.000001 * (float)sen_valmag[i].val2 ) ;
            }

            sumaccnony = sqrt((AccBuffer[0]*AccBuffer[0])+(AccBuffer[2]*AccBuffer[2]));
            sumaccnonx = sqrt((AccBuffer[1]*AccBuffer[1])+(AccBuffer[2]*AccBuffer[2]));

            sinconsty = AccBuffer[2]/sumaccnony;
            cosconsty = AccBuffer[0]/sumaccnony;
            sinconstx = AccBuffer[1]/sumaccnonx;
            cosconstx = AccBuffer[2]/sumaccnonx;
            if(sinconsty >= 0)
            {
                if(cosconsty <= 0)
                    consty = acos(cosconsty)*180/PI-90;
                else
                    consty = acos(cosconsty)*180/PI + 270;
            }
            else
            {
                consty =270 - acos(cosconsty)*180/PI;
            }
                
            if(sinconstx >= 0)
            {
                    constx = 360- acos(cosconstx)*180/PI ;
            }
            else
                constx = acos(cosconstx)*180/PI;
            
     
 
// //////////////////////////////////////////////bme680
//             // printf("enter into bme\n");
            sensor_sample_fetch(bme);
            sensor_channel_get(bme,SENSOR_CHAN_AMBIENT_TEMP,&tem);
            sensor_channel_get(bme,SENSOR_CHAN_PRESS,&pre);
//             sensor_channel_get(bme,SENSOR_CHAN_GAS_RES,&gas);
            sensor_channel_get(bme,SENSOR_CHAN_HUMIDITY,&HUM);
            
            printf("PRESS: %d.%d \n",pre.val1,pre.val2);
            printf("TEMP : %d.%d \n",tem.val1,tem.val2);
//             printf("GAS  : %d.%d \n",gas.val1,gas.val2);
            printf("HUM  : %d.%d \n\n",HUM.val1,HUM.val2);

//             //////////////////////////////////////////usart
// 			printf("usart\n");
			memset(Rx_Buffer,0,sizeof(Rx_Buffer));
            num = 0;//max = 37;
			
            for(int i = 0;i<8;i++)
            {
                uart_poll_out(uart_dev,send[i]);
            }
			int everynum = 0;//
			bool drop = false;
            while (num < 37)
            {
				everynum++;
                printf("num:%d\n",num);
                ret = uart_poll_in(uart_dev, &recvChar);
                if(ret == 0)
                {
                    Rx_Buffer[num++] = recvChar;
					everynum = 0;
                }
				if(num == 36){
						for(int i = 0;i<num;i++){
							printf("%d ",Rx_Buffer[i]);
						}
				}
				if(everynum >= 50){
					drop = true;
					break;
				}
                
            }
// 			printf("\n");
//             /////////////////////usart crc
		if(!drop)
		{
            after = crc16bitbybit(Rx_Buffer, 35);
	        hou = after;
	        qian = after>>8;


			
			
            if(hou == Rx_Buffer[35] && qian == Rx_Buffer[36]){
                Voltage_data=(((unsigned long)(Rx_Buffer[3]))<<24) |(((unsigned long)(Rx_Buffer[4]))<<16) |(((unsigned long)(Rx_Buffer[5]))<<8)|Rx_Buffer[6];
                Current_data=(((unsigned long)(Rx_Buffer[7]))<<24) |(((unsigned long)(Rx_Buffer[8]))<<16) |(((unsigned long)(Rx_Buffer[9]))<<8)|Rx_Buffer[10];
                Power_data  =(((unsigned long)(Rx_Buffer[11]))<<24)|(((unsigned long)(Rx_Buffer[12]))<<16)|(((unsigned long)(Rx_Buffer[13]))<<8)|Rx_Buffer[14];
                Energy_data =(((unsigned long)(Rx_Buffer[15]))<<24)|(((unsigned long)(Rx_Buffer[16]))<<16)|(((unsigned long)(Rx_Buffer[17]))<<8)|Rx_Buffer[18];
                Pf_data     =(((unsigned long)(Rx_Buffer[19]))<<24)|(((unsigned long)(Rx_Buffer[20]))<<16)|(((unsigned long)(Rx_Buffer[21]))<<8)|Rx_Buffer[22];
                CO2_data    =(((unsigned long)(Rx_Buffer[23]))<<24)|(((unsigned long)(Rx_Buffer[24]))<<16)|(((unsigned long)(Rx_Buffer[25]))<<8)|Rx_Buffer[26]; 
                temperature =(((unsigned long)(Rx_Buffer[27]))<<24)|(((unsigned long)(Rx_Buffer[28]))<<16)|(((unsigned long)(Rx_Buffer[29]))<<8)|Rx_Buffer[30]; 
                HZ          =(((unsigned long)(Rx_Buffer[31]))<<24)|(((unsigned long)(Rx_Buffer[32]))<<16)|(((unsigned long)(Rx_Buffer[33]))<<8)|Rx_Buffer[34];
                // printf("Voltage_data:%f\n",(float)Voltage_data/10000.0);
                printf("Current_data:%f\n",(float)Current_data/10000.0);
                // printf("Power_data:%f\n",(float)Power_data/10000.0);
                // printf("Energy_data:%f\n",(float)Energy_data/10000.0);
                // printf("Pf_data:%f\n",(float)Pf_data/1000.0);
                // printf("CO2_data:%f\n",(float)CO2_data/10000.0);
                 printf("temperature:%f\n",(float)temperature/100.0);
                // printf("HZ:%f\n",(float)HZ/100.0);
                num = 0;
                for(int i = 0;i<37;i++)
                {
                    // printf("%d ",Rx_Buffer[i]);
                    Rx_Buffer[i] = 0x0;
                }
                // printf("\n");
            }else{
				printf("crc failed\n");
			}
		}else{
			Current_data = 0.0;
			printf("drop!!!!!!!!!!!!!!\n");
		}



			memset(CoapCurrent,0,sizeof(CoapCurrent));
            sprintf(CoapCurrent, "0 %f", (float)Current_data/10000.0);
            ret = send_simple_coap_msgs_and_wait_for_reply(2,CoapCurrent);
			// printf("CoapCurrent:%s\n",CoapCurrent);

			memset(Coap1281Temp,0,sizeof(Coap1281Temp));
            sprintf(Coap1281Temp, "0 %f", (float)temperature/100.0);
            ret = send_simple_coap_msgs_and_wait_for_reply(1,Coap1281Temp);
// 			printf("Coap1281Temp:%s\n",Coap1281Temp);

			memset(CoapBMETemp,0,sizeof(CoapBMETemp));
			sprintf(CoapBMETemp,"0 %f",((float)tem.val1 + 0.000001 * (float)tem.val2));
			ret = send_simple_coap_msgs_and_wait_for_reply(5,CoapBMETemp);
			// printf("CoapBMETemp:%s\n",CoapBMETemp);

			memset(Coapxandy,0,sizeof(Coapxandy));
			int intconsty = floattoint(consty);
			int intconstx = floattoint(constx);
			sprintf(Coapxandy,"%d %d",intconsty,intconstx);
			ret = send_simple_coap_msgs_and_wait_for_reply(8,Coapxandy);
			// printf("Coapxandy:%s\n",Coapxandy);

			memset(CoapBMEPress,0,sizeof(CoapBMEPress));
			sprintf(CoapBMEPress,"0 %f",((float)pre.val1 + 0.000001 * (float)pre.val2));
			ret = send_simple_coap_msgs_and_wait_for_reply(4,CoapBMEPress);
			// printf("CoapBMEPress:%s\n",CoapBMEPress);

			memset(CoapBMEHum,0,sizeof(CoapBMEHum));
			sprintf(CoapBMEHum,"0 %f",((float)HUM.val1 + 0.000001 * (float)HUM.val2));
			ret = send_simple_coap_msgs_and_wait_for_reply(7,CoapBMEHum);
			// printf("CoapBMEHum:%s\n",CoapBMEHum);

// 			// printf("consty:%d\n",floattoint(consty));
//             // printf("constx:%d\n",floattoint(constx));
			count++;
			printf("%d\n",count);
             k_sleep(100);
        }















// unsigned char send[8] = {0x01,0x03,0x00,0x48,0x00,0x08,0xC4,0x1A};
//     	struct device *uart_dev = device_get_binding("UART_2");
//         if (uart_dev == NULL)
//         {
//             printf("NULL\n");
//         }
//         else 
//         {
//             printf("NOT NULL\n");
//             /* code */
//         }

//     while(1)
//     {
//         unsigned char Rx_Buffer[40];//35 36 check
//         int num = 0;//max = 37;
//         for(int i = 0;i<8;i++)
//         {
//             uart_poll_out(uart_dev,send[i]);
//         }
//         while (num < 37)
//         {
//             unsigned char  recvChar;
//             int ans = uart_poll_in(uart_dev, &recvChar);
//             if(ans == 0)
//             {
//                 Rx_Buffer[num++] = recvChar;
//             }
//             /* code */
//         }
//         unsigned long Voltage_data,Current_data,Power_data,Energy_data,Pf_data,CO2_data,temperature,HZ;
//         Voltage_data=(((unsigned long)(Rx_Buffer[3]))<<24) |(((unsigned long)(Rx_Buffer[4]))<<16) |(((unsigned long)(Rx_Buffer[5]))<<8)|Rx_Buffer[6];
//         Current_data=(((unsigned long)(Rx_Buffer[7]))<<24) |(((unsigned long)(Rx_Buffer[8]))<<16) |(((unsigned long)(Rx_Buffer[9]))<<8)|Rx_Buffer[10];
//         Power_data  =(((unsigned long)(Rx_Buffer[11]))<<24)|(((unsigned long)(Rx_Buffer[12]))<<16)|(((unsigned long)(Rx_Buffer[13]))<<8)|Rx_Buffer[14];
//         Energy_data =(((unsigned long)(Rx_Buffer[15]))<<24)|(((unsigned long)(Rx_Buffer[16]))<<16)|(((unsigned long)(Rx_Buffer[17]))<<8)|Rx_Buffer[18];
//         Pf_data     =(((unsigned long)(Rx_Buffer[19]))<<24)|(((unsigned long)(Rx_Buffer[20]))<<16)|(((unsigned long)(Rx_Buffer[21]))<<8)|Rx_Buffer[22];
//         CO2_data    =(((unsigned long)(Rx_Buffer[23]))<<24)|(((unsigned long)(Rx_Buffer[24]))<<16)|(((unsigned long)(Rx_Buffer[25]))<<8)|Rx_Buffer[26]; 
//         temperature =(((unsigned long)(Rx_Buffer[27]))<<24)|(((unsigned long)(Rx_Buffer[28]))<<16)|(((unsigned long)(Rx_Buffer[29]))<<8)|Rx_Buffer[30]; 
//         HZ          =(((unsigned long)(Rx_Buffer[31]))<<24)|(((unsigned long)(Rx_Buffer[32]))<<16)|(((unsigned long)(Rx_Buffer[33]))<<8)|Rx_Buffer[34];
//         printf("Voltage_data:%f\n",(float)Voltage_data/10000.0);
//         printf("Current_data:%f\n",(float)Current_data/10000.0);
//         printf("Power_data:%f\n",(float)Power_data/10000.0);
//         printf("Energy_data:%f\n",(float)Energy_data/10000.0);
//         printf("Pf_data:%f\n",(float)Pf_data/1000.0);
//         printf("CO2_data:%f\n",(float)CO2_data/10000.0);
//         printf("temperature:%f\n",(float)temperature/100.0);
//         printf("HZ:%f\n",(float)HZ/100.0);
//         num = 0;
//         for(int i = 0;i<37;i++)
//         {
//             printf("%d ",Rx_Buffer[i]);
//             Rx_Buffer[i] = 0x0;
//         }
//         printf("\n");
//         k_sleep(1000);
        
//     }


        /*
        *   read test
        */
        // unsigned char  recvChar;
        //  while (1) {
        //     int ans = uart_poll_in(uart_dev, &recvChar);
        //     if(ans == 0)
        //     {
        //         printf("%u ",recvChar);
        //     }

        // }
        /*
        *   send test
        */
        // unsigned char b[37] = {0x01 ,0x03,0x20,0x00,0x21,0x8D,0xD8,0x00,0x01,0x38,0x75,0x01,0x0C,0x63,0x08,0x00,0x00,0x00,0x5A,0x00,0x00,0x03,0xE8,0x00,0x00,0x00,0x59,0x00,0x00,0x0C,0xCB,0x00,0x00,0x13,0x88,0x1B,0xC2};
	    // char tx_data[38];
	    // for (int i = 0; i < 37; i++)
	    // {
	    // 	tx_data[i] = b[i];
	    // }
        // while(1)
        // {
        // for(int i = 0;i<37;i++)
        // {
        //     uart_poll_out(uart_dev,b[i]);
        // }
        // k_sleep(1000);
        // }


}
