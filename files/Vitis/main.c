/*
 * Project:
 * 		Using poling to sent RTOS
 * Design: /design_Ultra96_MicroBlaze_wrapper_v5
 *
 */

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/* Xilinx includes. */
#include "xil_printf.h"
#include "xparameters.h"
#include "xuartps.h"
/* GPIO includes*/
#include "xil_io.h"
#include "xgpio.h"

/* microRos includes */
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int64.h>
#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/image.h>

/*-----------------------------------------------------------*/

/* Hello World defines*/
#define DELAY_1_SECOND			1000UL

/* GPIO defines*/
#define GPIO_DEVICE_ID_OUT		XPAR_GPIO_0_DEVICE_ID
#define LED						0x01
#define LED_CH					1
#define LED_DEL					10000000

#define GPIO_DEVICE_ID_IN		XPAR_GPIO_1_DEVICE_ID
#define BUTTON_VOLTAGE			0x02
#define BUTTON					0x03
#define BUTTON_CH				1

/*-----------------------------------------------------------*/

/* GPIO variables*/
XGpio Gpio_out;
XGpio Gpio_in;
u32 button_red_1, button_red_2;
static TaskHandle_t toggleLEDTask;

/*-----------------------------------------------------------*/

/*LED tasks */
static void toggleLED( void *pvParameters );

/*-----------------------------------------------------------*/

// sensor_msg
uint8_t coffee[] = {
		  255 , 255 , 0   , 255 , 0   , 255 , 255 , 255  ,
		  255 , 0   , 255 , 0   , 255 , 255 , 255 , 255  ,
		  255 , 255 , 0   , 255 , 0   , 255 , 255 , 255  ,
		  0   , 0   , 0   , 0   , 0   , 0   , 0   , 0    ,
		  0   , 255 , 255 , 255 , 255 , 0   , 255 , 0    ,
		  0   , 255 , 255 , 255 , 255 , 0   , 0   , 0    ,
		  0   , 255 , 255 , 255 , 255 , 0   , 255 , 255  ,
		  255 , 0   , 0   , 0   , 0   , 255 , 255 , 255  ,
		};

uint8_t time[] = {
		  255 , 255 , 0   , 0   , 0   , 0   , 255 , 255  ,
		  255 , 0   , 255 , 255 , 255 , 255 , 0   , 255  ,
		  0   , 255 , 255 , 0   , 255 , 255 , 255 , 0    ,
		  0   , 255 , 255 , 0   , 255 , 255 , 255 , 0    ,
		  0   , 255 , 255 , 0   , 0   , 0   , 255 , 0    ,
		  0   , 255 , 255 , 255 , 255 , 255 , 255 , 0    ,
		  255 , 0   , 255 , 255 , 255 , 255 , 0   , 255  ,
		  255 , 255 , 0   , 0   , 0   , 0   , 255 , 255  ,
		};


#define img_encoding "rgb8" // Options: "mono8"
#define large_img_length 128
#define large_img_width 128
#define num_color 3
#define large_img_step large_img_width*num_color

uint8_t large_coffee[large_img_length*large_img_width*num_color] = {0};
uint8_t large_time[large_img_length*large_img_width*num_color] = {0};

/*-----------------------------------------------------------*/

#define MICHAL 0

#define TIMER_ID	1
#define DELAY_10_SECONDS	10000UL
#define DELAY_1_SECOND		1000UL
#define DELAY_60_SECOND		60000UL
#define DELAY_30_SECOND		30000UL
#define DELAY_10_SECOND		10000UL

#define DELAY_10_MS			10UL
#define TIMER_CHECK_THRESHOLD	9

#define UART_DEVICE_ID                  XPAR_XUARTPS_0_DEVICE_ID
#define THREAD_MIRCOROS_STACKSIZE 3000  // 12kb

#define MICROROS_TRANSPORTS_FRAMING_MODE 1
#define MICROROS_TRANSPORTS_PACKET_MODE 0

/*-----------------------------------------------------------*/
static void microros_msg_pub(void *pvParameters);
static void microros_thread_custom(void *pvParameters);
static void microros_init(void);

const TickType_t x1second = pdMS_TO_TICKS( DELAY_1_SECOND );
const TickType_t x60second = pdMS_TO_TICKS( DELAY_60_SECOND );
const TickType_t x30second = pdMS_TO_TICKS( DELAY_30_SECOND );
const TickType_t x10second = pdMS_TO_TICKS( DELAY_10_SECOND );
const TickType_t x10millisecond = pdMS_TO_TICKS( DELAY_10_MS );

/* microRos Prototype */
bool vitis_transport_open(struct uxrCustomTransport * transport);
bool vitis_transport_close(struct uxrCustomTransport * transport);
size_t vitis_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t vitis_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

#define node_name "vitis_publisher"

// TODO: handle topic name with ROS not microROS..
#define ros_pub_img_topic_name	"image_raw"
#define ros_pub_msg_topic_name	"img_detection"

int toggle = 0;

// TODO: chanhe name for xHandle
static TaskHandle_t xHandle = NULL;
static TaskHandle_t ros_msg_handle = NULL;

rcl_node_t img_node;


/*-----------------------------------------------------------*/

int main( void )
{
	/* 			 Create RGB image							*/
	for (int i = 0; i < large_img_width; i++) {
			large_coffee[i*3 + 0] = coffee[i]; // Red
			large_coffee[i*3 + 1] = 0; 								// Green
			large_coffee[i*3 + 2] = coffee[i]; // Blue
	}

	int Status;

	// INIT GPIO OUT
	Status = XGpio_Initialize(&Gpio_out, GPIO_DEVICE_ID_OUT);
	if( Status != XST_SUCCESS)
	{
		xil_printf("GPIO OUT INIT fail!");
	}
	XGpio_DiscreteWrite(&Gpio_out, LED_CH, BUTTON_VOLTAGE); // set GPIO to OFF maybe set to 0x01 for as voltage src

	// INIT GPIO IN
	Status = XGpio_Initialize(&Gpio_in, GPIO_DEVICE_ID_IN);
	if( Status != XST_SUCCESS)
	{
		xil_printf("GPIO IN INIT fail!");
	}

	microros_init();

	xTaskCreate( microros_thread_custom,			// Function to be called
				 ( const char * ) "ros_img",	// Name of task
				 THREAD_MIRCOROS_STACKSIZE,			// Stack size
				 NULL,								// Parameter to pass
				 tskIDLE_PRIORITY + 1,				// Task priority = 1
				 &xHandle );						// Task handle


	xTaskCreate( 	toggleLED, 					/* The function that implements the task. */
					( const char * ) "toggleLED", 		/* Text name for the task, provided to assist debugging only. */
					configMINIMAL_STACK_SIZE, 	/* The stack allocated to the task. */
					NULL, 						/* The task parameter is not used, so set to NULL. */
					tskIDLE_PRIORITY,			/* The task runs at the idle priority. */
					&toggleLEDTask );
//
	xTaskCreate( 	microros_msg_pub, 					/* The function that implements the task. */
					( const char * ) "ros_msg", 		/* Text name for the task, provided to assist debugging only. */
					THREAD_MIRCOROS_STACKSIZE, 	/* The stack allocated to the task. */
					NULL, 						/* The task parameter is not used, so set to NULL. */
					tskIDLE_PRIORITY,			/* The task runs at the idle priority. */
					&ros_msg_handle );


	/* Start the tasks and timer running. */
	vTaskStartScheduler();

	/* If all is well, the scheduler will now be running, and the following line
	will never be reached.*/
	for( ;; );
}


/*-----------------------------------------------------------*/
static void toggleLED( void *pvParameters )
{
	const TickType_t x1second = pdMS_TO_TICKS( DELAY_1_SECOND );

	for( ;; )
	{
		/* Delay for 1 second. */
		vTaskDelay( x1second );

		u32 gpio_read = XGpio_DiscreteRead(&Gpio_out, LED_CH);
		XGpio_DiscreteWrite(&Gpio_out, LED_CH, gpio_read ^ LED);

		button_red_1 = XGpio_DiscreteRead(&Gpio_in, BUTTON_CH);
		xil_printf("%d \r\n", button_red_1);

//		vTaskResume(xHandle);

	}
}

/*----------------------------------------------------*/
static void microros_init( void )
{
	// Transport
	rmw_uros_set_custom_transport(
		MICROROS_TRANSPORTS_FRAMING_MODE, // Framing enable here
		(void *) NULL, //transport->args
		vitis_transport_open,
		vitis_transport_close,
		vitis_transport_write,
		vitis_transport_read);

	// Initialize allocator
	rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
	freeRTOS_allocator.allocate = microros_allocate;
	freeRTOS_allocator.deallocate = microros_deallocate;
	freeRTOS_allocator.reallocate = microros_reallocate;
	freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

	if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
		xil_printf("Error on default allocators (line %d)\n", __LINE__);
	}


//	rclc_support_t img_support;
//
//	// rcl_node_t img_node;
//
//	rcl_allocator_t img_allocator = rcl_get_default_allocator();
//
//	//create img_init_options
//	rcl_ret_t img_ret_init_options = rclc_support_init(&img_support, 0, NULL, &img_allocator);
//	if (img_ret_init_options != RCL_RET_OK){
//		xil_printf("Error init img_ret_init_options %d - \n", img_ret_init_options);
//	}
//
//	// create img_node
//	rcl_ret_t img_ret_init_node = rclc_node_init_default(&img_node, node_name, "", &img_support);
//	if (img_ret_init_node != RCL_RET_OK){
//		xil_printf("Error init img_ret_init_node %d - \n", img_ret_init_node);
//	}
//	else{
//		xil_printf("img_node created %d \n", img_node);
//	}

	// TODO: Retrun -1 ??
}

/*----------------------------------------------------*/
static void microros_msg_pub( void *pvParameters )
{
//	vTaskDelay(x30second);

	// std_msg
	rcl_publisher_t publisher;
	std_msgs__msg__Int32 msg;

	// create publisher
	rcl_ret_t ret_init_publisher = rclc_publisher_init_default(
		&publisher,
		&img_node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		ros_pub_msg_topic_name); //topic name

	if (ret_init_publisher != RCL_RET_OK){
		xil_printf("Error init publisher %d - \n", ret_init_publisher);
	}

	msg.data = 0;

	for(;;)
	{
		// std_msg
		rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
		if (ret != RCL_RET_OK)
		{
		  xil_printf("Error publishing (msg.data: %d)\n", msg.data);
		}
		else {
			xil_printf("publishing (msg.data: %d)\n", msg.data);
		}

		msg.data++;

		vTaskDelay(x1second);
	}
}

/*----------------------------------------------------*/
static void microros_thread_custom( void *pvParameters )
{
	rclc_support_t img_support;

	// rcl_node_t img_node;

	rcl_allocator_t img_allocator = rcl_get_default_allocator();

	//create img_init_options
	rcl_ret_t img_ret_init_options = rclc_support_init(&img_support, 0, NULL, &img_allocator);
	if (img_ret_init_options != RCL_RET_OK){
		xil_printf("Error init img_ret_init_options %d - \n", img_ret_init_options);
	}

	// create img_node
	rcl_ret_t img_ret_init_node = rclc_node_init_default(&img_node, node_name, "", &img_support);
	if (img_ret_init_node != RCL_RET_OK){
		xil_printf("Error init img_ret_init_node %d - \n", img_ret_init_node);
	}
	else{
		xil_printf("img_node created %d \n", img_node);
	}







	// TODO: update frame_id every cycle

	char frame_id[] = "my ROS img";

	// TODO: make this smart using len of id
	int frame_id_size = 20;
	int frame_id_cap = 20;

	char encoding[] = img_encoding;

	// TODO: use pointer parameter for image in pvParameters??
	rosidl_runtime_c__uint8__Sequence ros_pub_img;
	ros_pub_img.data = large_coffee;									// Start address of data
	ros_pub_img.size = large_img_length*large_img_width*num_color;
	ros_pub_img.capacity = large_img_length*large_img_width*num_color;

	rosidl_runtime_c__String img_frame_id;
	img_frame_id.data = frame_id;
	img_frame_id.size = frame_id_size;
	img_frame_id.capacity = frame_id_cap;

	// TODO: make this smart using len of id
	rosidl_runtime_c__String ros_img_encoding;
	ros_img_encoding.data = encoding;
	ros_img_encoding.size = 10;
	ros_img_encoding.capacity = 10;

	// create publisher
	rcl_publisher_t img_publisher;
	rcl_ret_t img_ret_init_publisher = rclc_publisher_init_default(
		&img_publisher,
		&img_node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Image),
		ros_pub_img_topic_name); //topic name
	if (img_ret_init_publisher != RCL_RET_OK){
		xil_printf("Error img_init publisher %d - \n", img_ret_init_publisher);
	}

	sensor_msgs__msg__Image img_msg;
	img_msg.data.size = large_img_length*large_img_width*num_color;
	img_msg.data = ros_pub_img;
	img_msg.header.frame_id = img_frame_id;
	img_msg.encoding = ros_img_encoding;
	img_msg.height = large_img_length;
	img_msg.width = large_img_width;
	img_msg.step = large_img_step;


	for(;;)
	{
		// sensor_msg
		rcl_ret_t img_ret = rcl_publish(&img_publisher, &img_msg, NULL);
		if (img_ret != RCL_RET_OK)
		{
		  xil_printf("Error publishing (img_msg.data: %d)\n", img_msg.data);
		}
		else {
			xil_printf("publishing (img_msg.data: %d)\n", img_msg.data);
		}

		vTaskDelay(x10second);
//		vTaskSuspend(NULL);
	}
}



