#include <asf.h>
#include <delay.h>#define TCC_MODE_OTMX_DTI

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
	/* Initialize the PWM                                                   */
	/* We're using TCC0 pin PA04 and TCC1 pin PA06                          */
	/************************************************************************/
	uint8_t pwm_period = 112;
	
	struct tcc_config tcc0_cfg;
	tcc_get_config_defaults(&tcc0_cfg, TCC0);
	tcc0_cfg.counter.clock_prescaler = TCC_CLOCK_PRESCALER_DIV1;
	tcc0_cfg.counter.period = pwm_period;
	// output low when count is greater than CCx
	tcc0_cfg.compare.wave_generation = TCC_WAVE_GENERATION_SINGLE_SLOPE_PWM;
	//tcc0_cfg.counter.count = 66; // Offset the first PWM pulse
	
	tcc0_cfg.compare.match[0] = pwm_period / 2;
	tcc0_cfg.pins.enable_wave_out_pin[0] = true;
	tcc0_cfg.pins.wave_out_pin[0]        = PIN_PA04E_TCC0_WO0;
	tcc0_cfg.pins.wave_out_pin_mux[0]    = MUX_PA04E_TCC0_WO0;
	
	tcc0_cfg.compare.match[1] = pwm_period / 2;
	tcc0_cfg.pins.enable_wave_out_pin[1] = true;
	tcc0_cfg.pins.wave_out_pin[1]        = PIN_PA05E_TCC0_WO1;
	tcc0_cfg.pins.wave_out_pin_mux[1]    = MUX_PA05E_TCC0_WO1;
	
	struct tcc_module tcc0_instance;
	tcc_init(&tcc0_instance, TCC0, &tcc0_cfg);
	
	struct tcc_config tcc1_cfg;
	tcc_get_config_defaults(&tcc1_cfg, TCC1);
	tcc1_cfg.counter.clock_prescaler = TCC_CLOCK_PRESCALER_DIV1;
	tcc1_cfg.counter.period = pwm_period;
	tcc1_cfg.compare.wave_generation = TCC_WAVE_GENERATION_SINGLE_SLOPE_PWM;
	
	tcc1_cfg.compare.match[0] = pwm_period / 2;
	tcc1_cfg.pins.enable_wave_out_pin[0] = true;
	tcc1_cfg.pins.wave_out_pin[0]        = PIN_PA06E_TCC1_WO0;
	tcc1_cfg.pins.wave_out_pin_mux[0]    = MUX_PA06E_TCC1_WO0;
	
	tcc1_cfg.compare.match[1] = pwm_period / 2;
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
		measured_voltage1 = measure_voltage(&adc0_instance, ADC_POSITIVE_INPUT_PIN1, 2.0);
		measured_voltage2 = measure_voltage(&adc0_instance, ADC_POSITIVE_INPUT_PIN15, 2.0);
	    duty_cycle =( (measured_voltage1 / (measured_voltage1 + measured_voltage2)) * pwm_period)/2;
		duty_cycle2 = duty_cycle ;
		
		
		TCC0->CC[0].reg = duty_cycle;
		TCC0->CC[1].reg = duty_cycle;
		

		TCC1->CC[0].reg = duty_cycle;
		TCC1->CC[1].reg = duty_cycle;
		

		TCC0->COUNT.reg = 0;
		TCC1->COUNT.reg = offset;
		 
	}
}
