#include <linux/module.h>              // Core header for loading LKMs into the kernel
#include <linux/of_device.h>          // For device tree matching
#include <linux/kernel.h>             // Contains types, macros, functions for the kernel
#include <linux/timekeeping.h>        // For getting kernel time
#include <linux/ktime.h>              // For high-resolution timestamps
#include <linux/moduleparam.h>        // For module parameters
#include <linux/hrtimer.h>            // High-resolution timers
#include <linux/interrupt.h>          // For IRQ handling
#include <linux/of.h>                 // Device tree parsing
#include <linux/gpio/consumer.h>      // GPIO consumer API
#include <linux/platform_device.h>    // Platform driver definitions
#include <linux/interrupt.h>          // (duplicated include) For IRQ macros/functions



/**
 * Code from:  Haochen(Jason) Zhang, Ye Zhou, Konstantinos Alexopoulos
 * Class: COMP 424 ELEC 424/553 001
 * Final Project: Autonomous Car
 * Fall 2023
 *
 * Interrupt code derived from:
 * https://github.com/Johannes4Linux/Linux_Driver_Tutorial/blob/main/11_gpio_irq/gpio_irq.c
 */

// Global variables
unsigned int irq_number;                      // IRQ number assigned to encoder GPIO
static struct gpio_desc *my_enc;              // GPIO descriptor for encoder pin
static volatile u64 prev_time;                // Timestamp of previous interrupt
static volatile u64 curr_time;                // Timestamp of current interrupt

static int encoder_val;                       // Exposed variable for measured time between encoder ticks
module_param(encoder_val, int, 0644);         // Make encoder_val readable/writable from sysfs

// Interrupt handler for encoder pin
static irq_handler_t gpio_irq_handler(unsigned int irq, void *dev_id, struct pt_regs *regs) {

	// Read current value of encoder GPIO (should be 1 on rising edge)
	int enc_val = gpiod_get_value(my_enc); 

	// Get current high-resolution time
	curr_time = ktime_get_ns();
	unsigned long elapsed_time = curr_time - prev_time;

	// If rising edge and debounce threshold is satisfied (>1 ms)
	if (enc_val == 1 && elapsed_time > 1000000) {
		prev_time = curr_time;                 // Update last known interrupt time
		encoder_val = elapsed_time;            // Store elapsed time globally
		printk(KERN_INFO "irq_handler - timing detected: %lu", elapsed_time);
	}
	
	return (irq_handler_t) IRQ_HANDLED;        // Notify kernel interrupt has been handled
}


// Probe function called when the device is matched via device tree
static int enc_probe(struct platform_device *pdev)
{
	struct device *dev;
	dev = &pdev->dev;                         // Get pointer to device struct

	printk("enc_probe - RUNNING v5\n");

	// Request GPIO descriptor for encoder using label "encoder"
	my_enc = gpiod_get(dev, "encoder", GPIOD_IN);
	if(IS_ERR(my_enc)) {
		printk("enc_probe - could not gpiod_get for encoder\n");
		return -1;
	}
	
	// Convert GPIO descriptor to IRQ number
	irq_number = gpiod_to_irq(my_enc);

	// Set debounce interval to 1 ms (1,000,000 ns)
	gpiod_set_debounce(my_enc, 1000000);
	
	// Request IRQ for rising edge triggers on encoder GPIO
	if(request_irq(irq_number, (irq_handler_t) gpio_irq_handler, IRQF_TRIGGER_RISING, "my_gpio_irq", NULL) != 0){
		printk("Error!\nCannot request interrupt nr.: %d\n", irq_number);
		return -1;
	}

	// Initialize previous time for first interrupt measurement
	prev_time = ktime_get_ns();

	printk("enc_probe - SUCCESS\n");
	return 0;
}

// Cleanup function when the driver is removed
static void enc_remove(struct platform_device *pdev)
{
	// Free IRQ associated with encoder GPIO
	free_irq(irq_number, NULL);
	printk("enc_remove - Freed interrupt & Removing\n");
	return;
}

// Device tree match table used to bind platform driver to device
static struct of_device_id matchy_match[] = {
    { .compatible = "a_second_name", },       // Must match device tree "compatible" property
	{},
};

// Platform driver structure
static struct platform_driver my_driver = {
	.probe	 = enc_probe,                     // Called on device detection
	.remove	 = enc_remove,                   // Called on device removal
	.driver	 = {
	       .name  = "The Rock: this name doesn't even matter", // Kernel-internal name
	       .owner = THIS_MODULE,             // Indicates this module owns the driver
	       .of_match_table = matchy_match,   // Device tree match table
	},
};

// Register platform driver
module_platform_driver(my_driver);

// Module metadata
MODULE_ALIAS("platform:my_driver");
MODULE_DESCRIPTION("424's finest");          
MODULE_AUTHOR("UlisesM, LiamW, DevinG, JohnDavidV");
MODULE_LICENSE("GPL v2");                    