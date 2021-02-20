#include <asf.h>
#include <delay.h>

//Create a module software instance structure for the USART module
//to store the USART driver state while it is in use.
struct usart_module usart_instance;

//Create module software instance structures for DMA resources
//to store the DMA resource state while it is in use.
struct dma_resource usart_dma_resource_rx;
struct dma_resource usart_dma_resource_tx;

// Everything that comes from the PC goes into input_buffer
#define INPUT_BUF_SIZE 1
static uint16_t input_buffer[INPUT_BUF_SIZE];

// Everything that goes from the MCU to the PC goes into this buffer
#define OUTPUT_BUF_SIZE 2
static uint16_t output_buffer[OUTPUT_BUF_SIZE];
bool g_data_valid = false;
bool g_enable_output = true;
uint8_t g_pwm_period = 112;


//Create DMA transfer descriptors for RX/TX
COMPILER_ALIGNED(16)
DmacDescriptor example_descriptor_rx;
DmacDescriptor example_descriptor_tx;

// Configure DMA
// Create a callback function of receiver done
static void transfer_done_rx( const struct dma_resource* const resource ) {
	g_data_valid = true;
	dma_start_transfer_job(&usart_dma_resource_rx);
}

//Create a DMA resource configuration structure
static void configure_dma_resource_rx(struct dma_resource *resource)
{
	struct dma_resource_config config;
	
	//Initialize the DMA resource configuration struct with the module's default values
	dma_get_config_defaults(&config);
	
	//Set extra configurations for the DMA resource.
	//It is using peripheral trigger, SERCOM Tx empty trigger and
	//trigger causes a beat transfer in this example
	config.peripheral_trigger = SERCOM3_DMAC_ID_RX;
	config.trigger_action = DMA_TRIGGER_ACTION_BEAT;
	
	//Allocate a DMA resource with the configurations
	dma_allocate(resource, &config);
}

// Initialize the DMA transfer descriptor configuration struct with the module's default values.
static void setup_transfer_descriptor_rx(DmacDescriptor *descriptor)
{
	struct dma_descriptor_config descriptor_config;
	dma_descriptor_get_config_defaults(&descriptor_config);
	
	//Set the specific parameters for a DMA transfer with transfer size, source address, destination address
	descriptor_config.beat_size = DMA_BEAT_SIZE_HWORD;
	descriptor_config.src_increment_enable = false;
	descriptor_config.block_transfer_count = INPUT_BUF_SIZE;
	descriptor_config.destination_address = (uint32_t)input_buffer + sizeof(input_buffer);
	descriptor_config.source_address = (uint32_t)(&usart_instance.hw->USART.DATA.reg);
	
	//Create the DMA transfer descriptor
	dma_descriptor_create(descriptor, &descriptor_config);
}


static void configure_dma_resource_tx(struct dma_resource *resource)
{
	//Create a DMA resource configuration structure for tx
	struct dma_resource_config config;
	
	//Initialize the DMA resource configuration struct with the module's default values
	dma_get_config_defaults(&config);
	config.peripheral_trigger = SERCOM3_DMAC_ID_TX;
	config.trigger_action = DMA_TRIGGER_ACTION_BEAT;
	dma_allocate(resource, &config);
}
static void setup_transfer_descriptor_tx(DmacDescriptor *descriptor)
{
	struct dma_descriptor_config descriptor_config;
	dma_descriptor_get_config_defaults(&descriptor_config);
	descriptor_config.beat_size = DMA_BEAT_SIZE_HWORD;
	descriptor_config.dst_increment_enable = false;
	descriptor_config.block_transfer_count = OUTPUT_BUF_SIZE;
	descriptor_config.source_address = (uint32_t)output_buffer + sizeof(output_buffer);
	descriptor_config.destination_address = (uint32_t)(&usart_instance.hw->USART.DATA.reg);
	dma_descriptor_create(descriptor, &descriptor_config);
}
 
void check_for_commands() {
	if (g_data_valid) {
		g_data_valid = false;
		output_buffer[0] = 'o';
		output_buffer[1] = '\n';
			
		if (input_buffer[0] == 'u') {
			// Frequency goes up, this period gets smaller
			g_pwm_period -= 10;
			TCC0->PER.reg = g_pwm_period;
			TCC1->PER.reg = g_pwm_period;
			} else if (input_buffer[0] == 'd') {
				// Frequency goes down, thus period gets longer
				g_pwm_period += 10;
				TCC0->PER.reg = g_pwm_period;
				TCC1->PER.reg = g_pwm_period;
			} else if (input_buffer[0] == 's') {
				g_enable_output = false;
				port_pin_set_output_level(LED_0_PIN, true);
			} else if (input_buffer[0] == 'r') {
				g_enable_output = true;
				port_pin_set_output_level(LED_0_PIN, false);
			} else {
			output_buffer[0] = 'e';
		}
		dma_start_transfer_job(&usart_dma_resource_tx);
	}
}

// CONFIGURE USART
// Create a USART module configuration struct, which can be filled out to adjust the configuration of
// a physical USART peripheral
static void configure_usart(void) {
	struct usart_config config_usart;
	
	//Initialize the USART configuration struct with the module's default values
	usart_get_config_defaults(&config_usart);
	config_usart.baudrate = 9600;
	config_usart.mux_setting = EDBG_CDC_SERCOM_MUX_SETTING;
	config_usart.pinmux_pad0 = EDBG_CDC_SERCOM_PINMUX_PAD0;
	config_usart.pinmux_pad1 = EDBG_CDC_SERCOM_PINMUX_PAD1;
	config_usart.pinmux_pad2 = EDBG_CDC_SERCOM_PINMUX_PAD2;
	config_usart.pinmux_pad3 = EDBG_CDC_SERCOM_PINMUX_PAD3;
	
	//Configure the USART module with the desired settings, retrying while the driver is busy
	//until the configuration is stressfully set.
	while (usart_init(&usart_instance, EDBG_CDC_MODULE, &config_usart) != STATUS_OK);
	
	//Enable the USART module.
	usart_enable(&usart_instance);
}


static float measure_voltage(struct adc_module *adc, enum adc_positive_input analog_ch, float ref_voltage){
	
	adc_set_positive_input(adc, analog_ch);
	adc_start_conversion(adc);
	
	uint16_t adc_value;
	adc_read(adc, &adc_value);
	
	return (adc_value / 255.0) * ref_voltage;
}

int main (void) {
	// https://asf.microchip.com/docs/latest/search.html?device=saml21
	/************************************************************************/
	/* Increase CPU clock to 16MHz                                          */
	/************************************************************************/
	OSCCTRL->OSC16MCTRL.bit.FSEL = 0x3;
	
	/************************************************************************/
	/* Set internal voltage reference for ADC/SDADC/DAC                     */
	/* Family data sheet 23.8.7                                             */
	/************************************************************************/
	struct system_voltage_references_config voltage_reference_cfg;
	system_voltage_reference_get_config_defaults(&voltage_reference_cfg);
	voltage_reference_cfg.sel = SYSTEM_VOLTAGE_REFERENCE_2V0;
	system_voltage_reference_set_config(&voltage_reference_cfg);
	
	/************************************************************************/
	/* Enable UART communication                                            */
	/* Family data sheet 15.0.0                                             */
	/************************************************************************/
	struct port_config config_port_pin;
	port_get_config_defaults(&config_port_pin);
	config_port_pin.direction = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(LED_0_PIN, &config_port_pin);
	
	configure_usart();
	configure_dma_resource_rx(&usart_dma_resource_rx);
	configure_dma_resource_tx(&usart_dma_resource_tx);
	
	setup_transfer_descriptor_rx(&example_descriptor_rx);
	setup_transfer_descriptor_tx(&example_descriptor_tx);
	
	dma_add_descriptor(&usart_dma_resource_rx, &example_descriptor_rx);
	dma_add_descriptor(&usart_dma_resource_tx, &example_descriptor_tx);
	
	dma_register_callback(&usart_dma_resource_rx, transfer_done_rx, DMA_CALLBACK_TRANSFER_DONE);
	
	dma_enable_callback(&usart_dma_resource_rx, DMA_CALLBACK_TRANSFER_DONE);
	dma_enable_callback(&usart_dma_resource_tx, DMA_CALLBACK_TRANSFER_DONE);
	
	dma_start_transfer_job(&usart_dma_resource_rx);
	/************************************************************************/
	/* Initialize the PWM                                                   */
	/* We're using TCC0 pin PA04 and TCC1 pin PA06                          */
	/************************************************************************/
	
	struct tcc_config tcc0_cfg;
	tcc_get_config_defaults(&tcc0_cfg, TCC0);
	tcc0_cfg.counter.clock_prescaler = TCC_CLOCK_PRESCALER_DIV1;
	tcc0_cfg.counter.period = g_pwm_period;
	// output low when count is greater than CCx
	tcc0_cfg.compare.wave_generation = TCC_WAVE_GENERATION_SINGLE_SLOPE_PWM;
	//tcc0_cfg.counter.count = 66; // Offset the first PWM pulse
	
	tcc0_cfg.compare.match[0] = g_pwm_period / 2;
	tcc0_cfg.pins.enable_wave_out_pin[0] = true;
	tcc0_cfg.pins.wave_out_pin[0]        = PIN_PA04E_TCC0_WO0;
	tcc0_cfg.pins.wave_out_pin_mux[0]    = MUX_PA04E_TCC0_WO0;
	
	tcc0_cfg.compare.match[1] = g_pwm_period / 2;
	tcc0_cfg.pins.enable_wave_out_pin[1] = true;
	tcc0_cfg.pins.wave_out_pin[1]        = PIN_PA05E_TCC0_WO1;
	tcc0_cfg.pins.wave_out_pin_mux[1]    = MUX_PA05E_TCC0_WO1;
	
	struct tcc_module tcc0_instance;
	tcc_init(&tcc0_instance, TCC0, &tcc0_cfg);
	
	struct tcc_config tcc1_cfg;
	tcc_get_config_defaults(&tcc1_cfg, TCC1);
	tcc1_cfg.counter.clock_prescaler = TCC_CLOCK_PRESCALER_DIV1;
	tcc1_cfg.counter.period = g_pwm_period;
	tcc1_cfg.compare.wave_generation = TCC_WAVE_GENERATION_SINGLE_SLOPE_PWM;
	
	tcc1_cfg.compare.match[0] = g_pwm_period / 2;
	tcc1_cfg.pins.enable_wave_out_pin[0] = true;
	tcc1_cfg.pins.wave_out_pin[0]        = PIN_PA06E_TCC1_WO0;
	tcc1_cfg.pins.wave_out_pin_mux[0]    = MUX_PA06E_TCC1_WO0;
	
	tcc1_cfg.compare.match[1] = g_pwm_period / 2;
	tcc1_cfg.pins.enable_wave_out_pin[1] = true;
	tcc1_cfg.pins.wave_out_pin[1]        = PIN_PA07E_TCC1_WO1;
	tcc1_cfg.pins.wave_out_pin_mux[1]    = MUX_PA07E_TCC1_WO1;
	
	struct tcc_module tcc1_instance;
	tcc_init(&tcc1_instance, TCC1, &tcc1_cfg);
	tcc1_instance.hw->WAVE.bit.POL0 = !tcc1_instance.hw->WAVE.bit.POL0;
	tcc1_instance.hw->WAVE.bit.POL1 = !tcc1_instance.hw->WAVE.bit.POL1;
	
	
	/************************************************************************/
	/* Initialize the ADCs                                                  */
	/* We're using pins PA03 (AIN1) and PB07 (AIN15)                        */
	/************************************************************************/
	// Configure analog pins
	struct system_pinmux_config config;
	system_pinmux_get_config_defaults(&config);

	// input_pull is being set to SYSTEM_PINMUX_PIN_PULL_NONE to cancel out the
	// default pinmux_config struct setting of a pull-up enabled.
	config.input_pull   = SYSTEM_PINMUX_PIN_PULL_NONE;
	config.mux_position = 1;
	system_pinmux_pin_set_config(PIN_PA03, &config);
	system_pinmux_pin_set_config(PIN_PB07, &config);
	
	// Configure the ADC
	struct adc_config adc0_cfg;
	adc_get_config_defaults(&adc0_cfg);
	adc0_cfg.resolution = ADC_RESOLUTION_8BIT;

	struct adc_module adc0_instance;
	adc_init(&adc0_instance, ADC, &adc0_cfg);
	/************************************************************************/
	/* Enable all the peripherals with the configurations stated above      */
	/************************************************************************/
	adc_enable(&adc0_instance);
	tcc0_instance.hw->CTRLA.reg |= TCC_CTRLA_ENABLE;
	tcc1_instance.hw->CTRLA.reg |= TCC_CTRLA_ENABLE;


	uint8_t duty_cycle2;
	uint8_t duty_cycle;
	int8_t offset = 10; //66 Tested offset of a half a period
	float measured_voltage1;
	float measured_voltage2;
	bool offset_timer_zero = false;
	
	while (1) {
		check_for_commands();
		
		measured_voltage1 = measure_voltage(&adc0_instance, ADC_POSITIVE_INPUT_PIN1, 2.0);
		measured_voltage2 = measure_voltage(&adc0_instance, ADC_POSITIVE_INPUT_PIN15, 2.0);
		duty_cycle =( (measured_voltage1 / (measured_voltage1 + measured_voltage2)) * g_pwm_period)/2;
		
		if (!g_enable_output) {
			duty_cycle = 0;
		}

		TCC0->CC[0].reg = duty_cycle;
		TCC0->CC[1].reg = duty_cycle;
		
		TCC1->CC[0].reg = duty_cycle;
		TCC1->CC[1].reg = duty_cycle;
		
		TCC0->COUNT.reg = 0;
		TCC1->COUNT.reg = offset;
	}
}
