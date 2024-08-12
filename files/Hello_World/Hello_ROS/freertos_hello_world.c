/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
/* Xilinx includes. */
#include "xil_printf.h"
#include "xparameters.h"
#include "xuartps.h"
/* microRos includes */
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include <std_msgs/msg/int64.h>
#include <std_msgs/msg/int32.h>


#define TIMER_ID	1
#define DELAY_10_SECONDS	10000UL
#define DELAY_1_SECOND		1000UL
#define DELAY_10_MS			10UL
#define TIMER_CHECK_THRESHOLD	9
/*-----------------------------------------------------------*/
#define UART_DEVICE_ID                  XPAR_XUARTPS_0_DEVICE_ID
#define THREAD_MIRCOROS_STACKSIZE 3000  // 12kb

#define MICROROS_TRANSPORTS_FRAMING_MODE 1
#define MICROROS_TRANSPORTS_PACKET_MODE 0


static void microros_thread_custom(void *pvParameters);
const TickType_t x1second = pdMS_TO_TICKS( DELAY_1_SECOND );
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


/*-----------------------------------------------------------*/

static TaskHandle_t xHandle = NULL;



int main( void )
{
	const TickType_t x10seconds = pdMS_TO_TICKS( DELAY_10_SECONDS );

	xTaskCreate( microros_thread_custom,
				 ( const char * ) "GB",
				 THREAD_MIRCOROS_STACKSIZE,
				 NULL,
				 tskIDLE_PRIORITY + 1,
				 &xHandle );

	/* Start the tasks and timer running. */
	vTaskStartScheduler();

	/* If all is well, the scheduler will now be running, and the following line
	will never be reached.  If the following line does execute, then there was
	insufficient FreeRTOS heap memory available for the idle and/or timer tasks
	to be created.  See the memory management section on the FreeRTOS web site
	for more details. */
	for( ;; );
}
/*----------------------------------------------------*/
static void microros_thread_custom( void *pvParameters )
{
	// Transport
	rmw_uros_set_custom_transport(
		MICROROS_TRANSPORTS_FRAMING_MODE, // Framing enable here
		(void *) NULL, //transport->args
		vitis_transport_open,
		vitis_transport_close,
		vitis_transport_write,
		vitis_transport_read);


		rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
		freeRTOS_allocator.allocate = microros_allocate;
		freeRTOS_allocator.deallocate = microros_deallocate;
		freeRTOS_allocator.reallocate = microros_reallocate;
		freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

		if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
			xil_printf("Error on default allocators (line %d)\n", __LINE__);
		}

		// micro-ROS app
		rcl_publisher_t publisher;
		std_msgs__msg__Int32 msg;
		rclc_support_t support;
		rcl_allocator_t allocator;
		rcl_node_t node;

		allocator = rcl_get_default_allocator();

		//create init_options
		rcl_ret_t ret_init_support = rclc_support_init(&support, 0, NULL, &allocator);
		if (ret_init_support != RCL_RET_OK){
			xil_printf("Error init support %d - \n", ret_init_support);
		}

		// create node
		rclc_node_init_default(&node, "vitis_node", "", &support);
		xil_printf("node created %d \n", node);

		// create publisher
		rcl_ret_t ret_init_publisher = rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
		"vitis_publisher"); //topic name

		if (ret_init_publisher != RCL_RET_OK){
			xil_printf("Error init publisher %d - \n", ret_init_publisher);
		}

		msg.data = 0;

		for(;;)
		{
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
			//sleep(1);
		}
}
