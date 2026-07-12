#pragma once

#include "esphome.h"

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"

#include "driver/gpio.h"
#include "driver/timer.h"
#include "soc/rtc.h"
#include "soc/frc_timer_reg.h"
#include "driver/ledc.h"
#include <driver/adc.h>
#include <driver/dac.h>
#include "driver/uart.h"
#include <driver/spi_master.h>
#include "esp_timer.h"
#include <esp_task_wdt.h>

#include <soc/sens_reg.h>
#include <soc/sens_struct.h>
#include <soc/ledc_struct.h>
#include "soc/gpio_reg.h"

// Logging tags
static const char *TAG_WIFI = "wifi_sta";
static const char *TAG_MQTT = "mqtt";
static const char *TAG_UART = "uart_linky";

#define WIFI_MAXIMUM_RETRY (5)

#define PIN_LED ((gpio_num_t)2)	  // pin de la LED contrôle sur l'ESP32
#define PIN_UART ((gpio_num_t)4)  // pin à utiliser pour rerouter l'UART pour RS-485
#define PIN_TRIAC ((gpio_num_t)5) // pin qui contrôle la gachette du TRIAC/SSR
#define PIN_ZC ((gpio_num_t)18)	  // pin qui montre la détection de ZC. Doit être < 32 (accès direct HW)

#define SAMPLES_PER_CYCLE (200) // nombre de paires d'échantillons (I,V) par cycle.
#define ADC_BITS (12)
#define MAX_ADC_OUTPUT ((1 << ADC_BITS) - 1)
#define ADC_OFFSET (1 << (ADC_BITS - 1))
#define ADC_VOLT_CHANNEL ADC1_CHANNEL_0							  // PIN 36
#define ADC_AMPS_CHANNEL ADC1_CHANNEL_3							  // PIN 39
#define ADC_CALI_CHANNEL ADC1_CHANNEL_5							  // PIN 33
#define DAC_CALI_CHANNEL DAC_CHAN_0							  // PIN 25
#define DAC_BIASV_CHANNEL DAC_CHAN_1							  // PIN 26
#define TIMER_DIVIDER (2)										  // Division pour le timer échantillonage (80 / 2 = 40 MHz)
#define TIMER_SCALE_SEC (80000000ULL / TIMER_DIVIDER)		  // convertir compteur en secondes
#define TIMER_INTERVAL (TIMER_SCALE_SEC / 50 / SAMPLES_PER_CYCLE) // délai échantillonnage par cycle de 20ms
#define PWM_DUTY_BIT_DEPTH (10)									  // Résolution en bit de la charge pour le LEDC PWM controller
#define PWM_FREQUENCY (100)										  // Fréquence d'un 1/2 cycle  - TODO : changer  à la freq. mesurée
#define TRIAC_GATE_IMPULSE_CYCLES (10)							  // longueur de l'impulsion envoyée au TRIAC/SSR
#define TRIAC_GATE_QUIESCE_CYCLES (50)							  // temps min pour la gachette du TRIAC avant le prochain zc
#define LINKY_UART_NUM UART_NUM_2								  // Numéro de la Linky UART
#if CONFIG_LINKY_STANDARD
#define LINKY_BAUD (9600) // Linky mode standard: 9600 bauds
#else
#define LINKY_BAUD (1200) // Linky mode historique: 1200 bauds
#endif
#define LINKY_BUFFER_SIZE (1024) // Taille du buffer Linky. TODO: réduire au minimum
#define PATTERN_CHR_NUM (1)		 // Nombre de caractères dans le motif UART

// macro recalibration ADC par la table ADC_calibration
#define recalibrate(r) ((ADC_calibration[(r) >> 4] * (16 - ((r)&0xF))) + (ADC_calibration[((r) >> 4) + 1] * ((r)&0xF)))
// filtrage numérique simple
#define filter(o, n, decay) o = (((o) * (decay)) + ((n) * (1 - (decay))))

//-------------------------------------------------------------
// Conversion des lectures en unités du système international (SI)
//-------------------------------------------------------------
#define TRANSFORMERATIO (230.0 / 2.085)
#define VDIVIDERATIO (2.0)
#define VREF (3.3)
#define BIT2VOLTMULTIPLIER ((VREF / (1 << ADC_BITS)) * TRANSFORMERATIO * VDIVIDERATIO)
#define BURDENRESISTOR 26
#define ICTRATIO 1000
#define BIT2AMPSMULTIPLIER (((VREF / (1 << ADC_BITS)) / BURDENRESISTOR) * ICTRATIO)

//-------------------------------------------------------------
//      Variables globales
// ------------------------------------------------------------
// -- Linky UART --
QueueHandle_t uart_queue;
TaskHandle_t Linky; // Handle vers le buffer de l'UART Linky

// -- Échantillonnage --
unsigned short ADC_calibration[258];   // Données de calibration pour le convertisseur AD
TaskHandle_t SA;					   // Handle vers la tâche d'analyse des échantillons
unsigned short adc_biasV = ADC_OFFSET; // Offset pour le canal des tensions
unsigned short adc_biasI = ADC_OFFSET; // Offset pour le canal des courants
short phase_shift_cor = 1;			   // Correction de phase (prédiction linéaire)
bool prev_sign = true;				   // Signe du demi-cycle précédent
unsigned long gsumV2[2] = {0L, 0L};	   // Somme des V² par signe de demi-cycle, long car 24 bits sommé plus de 2⁹ fois
unsigned long gsumI2[2] = {0L, 0L};	   // Somme des I² par signe de demi-cycle, long car 24 bits sommé plus de 2⁹ fois
long gsumP[2] = {0L, 0L};			   // Somme des P actives par demi-cycle,   long car 24 bits sommé plus de 2⁹ fois
uint64_t alarm_value = TIMER_INTERVAL; // Valeur initiale du timer à interruption échantillonnage
uint32_t zc_time = 0;				   // Date du dernier zéro-crossing (en µS, pris sur le registre hw de l'esp timer)
uint32_t lastzc_time = 0;			   // Date du précédent zéro-crossing (en µS, pris sur le registre hw de l'esp timer)
float One_sec_Vrms = 230.00;		   // Moyenne glissante sur une seconde de Vrms
float One_sec_Irms = 0.0;			   // Moyenne glissante sur une seconde de Irms
float Buf_Vrms[100];				   // Buffer d'une seconde de Vrms pour calibrage par Linky
float Buf_Irms[100];				   // Buffer d'une seconde de Irms pour calibrage par Linky
unsigned Buf_rms_idx = 0;			   // Position courante dans les buffers Vrms et Irms
uint32_t apb_freq = 80000000;		   // Frequence APB
float P_F = 0.0;					   // Filtered active power measured by ICT * V
float cosf_F = 1.0;					   // Filtered Cos(phi)
float Vrms_F = 230.0;				   // Filtered RMS voltage
float R_est = 0.0;					   // Estimated resistance of the load (a purely resistive load is assumed)

// -- Contrôleur PI du TRIAC --
int command_mode = 2;		   // -1 = OFF, 0 = AUTO, 1 = ON, 2 = PWM
float command = 0.0;		   // Command sent to TRIAC in fraction of maximal energy
float set_point = 0.0;		   // Puissance active cible
const float PI_gain = 0.00002; // Gain du contrôleur PI
const float PI_tau = 1.0;	   // Délai du contrôleur PI

float energy2delay[101];

//-------------------------------------------------------------
//      Configuration UART Linky
// ------------------------------------------------------------
void linky_init_uart()
{
	uart_config_t uart_config = {
		.baud_rate = LINKY_BAUD,
		.data_bits = UART_DATA_7_BITS,
		.parity = UART_PARITY_EVEN,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
	};
	// Configure UART parameters
	ESP_ERROR_CHECK(uart_param_config(LINKY_UART_NUM, &uart_config));
	ESP_ERROR_CHECK(uart_set_pin(LINKY_UART_NUM, GPIO_NUM_17, GPIO_NUM_16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
	ESP_ERROR_CHECK(uart_driver_install(LINKY_UART_NUM, LINKY_BUFFER_SIZE * 2, 0, 20, &uart_queue, 0)); // No TX buffer
	uart_enable_pattern_det_baud_intr(LINKY_UART_NUM, 0x0D, PATTERN_CHR_NUM, 9, 0, 0);
	uart_pattern_queue_reset(LINKY_UART_NUM, 20);
}

static void linky_event_task(void *pvParameters)
{
	uart_event_t event;
	size_t buffered_size;
	uint8_t *dtmp = (uint8_t *)malloc(LINKY_BUFFER_SIZE);

	for (;;)
	{
		// Attente d'évènement UART
		if (xQueueReceive(uart_queue, (void *)&event, portMAX_DELAY))
		{
			bzero(dtmp, LINKY_BUFFER_SIZE);
			ESP_LOGI(TAG_UART, "Évènement UART[%d] :", LINKY_UART_NUM);
			switch (event.type)
			{
			case UART_DATA:
				ESP_LOGI(TAG_UART, "[Donnée UART]: %d", event.size);
				uart_read_bytes(LINKY_UART_NUM, dtmp, event.size, portMAX_DELAY);
				ESP_LOGI(TAG_UART, "[Évènmt UART]:");
				uart_write_bytes(LINKY_UART_NUM, (const char *)dtmp, event.size);
				break;
			case UART_FIFO_OVF:
				ESP_LOGI(TAG_UART, "Overflow FIFO UART");
				uart_flush_input(LINKY_UART_NUM);
				xQueueReset(uart_queue);
				break;
			case UART_BUFFER_FULL:
				ESP_LOGI(TAG_UART, "Ring buffer UART plein");
				uart_flush_input(LINKY_UART_NUM);
				xQueueReset(uart_queue);
				break;
			case UART_BREAK:
				ESP_LOGI(TAG_UART, "Rx break UART");
				break;
			case UART_PARITY_ERR:
				ESP_LOGI(TAG_UART, "Erreur de parité UART");
				break;
			case UART_FRAME_ERR:
				ESP_LOGI(TAG_UART, "Erreur de frame UART");
				break;
			case UART_PATTERN_DET:
				{
					uart_get_buffered_data_len(LINKY_UART_NUM, &buffered_size);
					int pos = uart_pattern_pop_pos(LINKY_UART_NUM);
					ESP_LOGI(TAG_UART, "[PATTERN UART DÉTECTÉ] pos: %d, buffer: %d", pos, buffered_size);
					if (pos == -1)
					{
						uart_flush_input(LINKY_UART_NUM);
					}
					else
					{
						uart_read_bytes(LINKY_UART_NUM, dtmp, pos, 100 / portTICK_PERIOD_MS);
						uint8_t pat[PATTERN_CHR_NUM + 1];
						memset(pat, 0, sizeof(pat));
						uart_read_bytes(LINKY_UART_NUM, pat, PATTERN_CHR_NUM, 100 / portTICK_PERIOD_MS);
						ESP_LOGI(TAG_UART, "lecture : %s", dtmp);
						ESP_LOGI(TAG_UART, "pattern : %s", pat);
					}
				}
				break;
			default:
				ESP_LOGI(TAG_UART, "Évènement UART type: %d", event.type);
				break;
			}
		}
	}
	free(dtmp);
	dtmp = NULL;
	vTaskDelete(NULL);
}

//-------------------------------------------------------------
//      Calcul de la table de conversion d'énergie en délai
// ------------------------------------------------------------
inline float psin2(float x)
{
	return (0.5 * (x - (sin(2 * x) / 2)));
}

void compute_e2d_table()
{
	double x1 = 0;
	double x2, x3;
	static const double tolerance = 1e-6;
	energy2delay[0] = 0.0;

	printf("Tabulation énergie-délai...");
	fflush(stdout);
	for (int i = 1; i <= 100; ++i)
	{
		double fx = i * (M_PI / 400);
		x3 = x1;
		while (psin2(x3) <= fx)
		{
			x1 = x3;
			x3 += (M_PI / 100);
		}
		x2 = (x1 + x3) / 2;
		while (fabs(psin2(x2) - fx) > tolerance)
		{
			if (psin2(x2) <= fx)
				x1 = x2;
			else
				x3 = x2;
			x2 = (x1 + x3) / 2;
		}
		energy2delay[i] = x2 / M_PI;
		x1 = x2;
	}
	printf("Ok.\n");
}

float energy_fraction_to_delay(float E_frac)
{
	E_frac = ((E_frac < 0.0) ? 0.0 : E_frac);
	E_frac = ((E_frac > 1.0) ? 1.0 : E_frac);
	E_frac = 1.0 - E_frac;

	if (E_frac <= 0.5)
	{
		E_frac *= 2.0;
		float alpha = (100 * E_frac) - floor(100 * E_frac);
		return ((1.0 - alpha) * energy2delay[(int)floor(E_frac * 100)] + alpha * energy2delay[(int)ceil(E_frac * 100)]);
	}
	else
	{
		E_frac = (1.0 - E_frac) * 2.0;
		float alpha = (100 * E_frac) - floor(100 * E_frac);
		return 1.0 - ((1.0 - alpha) * energy2delay[(int)floor(E_frac * 100)] + alpha * energy2delay[(int)ceil(E_frac * 100)]);
	}
}

void set_control(float cmd)
{
	if (cmd <= 0.001)
	{
		LEDC.channel_group[0].channel[0].conf0.sig_out_en = 0;
	}
	else if (cmd >= 0.999)
	{
		LEDC.channel_group[0].channel[0].duty.duty = (1 << (PWM_DUTY_BIT_DEPTH + 3)) - 1;
		LEDC.channel_group[0].channel[0].hpoint.hpoint = 0;
		LEDC.channel_group[0].channel[0].conf0.sig_out_en = 1;
		LEDC.channel_group[0].channel[0].conf1.duty_start = 1;
	}
	else
	{
		LEDC.channel_group[0].channel[0].duty.duty = TRIAC_GATE_IMPULSE_CYCLES << 4;
		LEDC.channel_group[0].channel[0].hpoint.hpoint = (unsigned)(energy_fraction_to_delay(cmd) * (1 << PWM_DUTY_BIT_DEPTH));
		LEDC.channel_group[0].channel[0].conf0.sig_out_en = 1;
		LEDC.channel_group[0].channel[0].conf1.duty_start = 1;
	}
}

//-------------------------------------------------------------
//      Analyse des statistiques d'échantillonnage
// ------------------------------------------------------------
void sample_analyzer(void *parameters)
{
	static uint32_t ulNotifiedValue;
	static BaseType_t xResult;
	static unsigned halfCycleCount = 0;
	static float Period_F = 10e-3;
	static float Pa_F = 0.0;
	static float integral = 0.0;
	float error;

	for (;;)
	{
		xResult = xTaskNotifyWait(pdFALSE, ULONG_MAX, &ulNotifiedValue, portMAX_DELAY);
		if (xResult == pdPASS)
		{
			LEDC.timer_group[0].timer[0].conf.rst = 1;
			LEDC.timer_group[0].timer[0].conf.rst = 0;

			unsigned short num_samples = (unsigned short)ulNotifiedValue;
			float P = (float)gsumP[1 - prev_sign] / num_samples * (BIT2VOLTMULTIPLIER * BIT2AMPSMULTIPLIER);
			filter(P_F, P, 0.98);

			if (command_mode == -1)
			{
				command = 0.0;
				error = 0;
			}
			else if (command_mode == 1)
			{
				command = 1.0;
				error = 0;
			}
			else if (command_mode == 2)
			{
				error = 0;
			}
			else
			{
				error = (set_point - P_F);
				integral += error;
				command = (PI_gain * error) + ((PI_gain / PI_tau) * integral);
			}

			if (command < 0.0)
			{
				command = 0.0;
				integral -= error;
			}
			else if (command > 1.0)
			{
				command = 1.0;
				integral -= error;
			}
			set_control(command);

			float delta_zc = zc_time - lastzc_time;
			filter(Period_F, delta_zc / apb_freq, 0.98);

			float Vrms = sqrt(((float)gsumV2[1 - prev_sign] / num_samples) * BIT2VOLTMULTIPLIER * BIT2VOLTMULTIPLIER);
			One_sec_Vrms -= Buf_Vrms[Buf_rms_idx];
			Buf_Vrms[Buf_rms_idx] = (Vrms / 100.0);
			float Irms = sqrt(((float)gsumI2[1 - prev_sign] / num_samples) * BIT2AMPSMULTIPLIER * BIT2AMPSMULTIPLIER);
			One_sec_Irms -= Buf_Irms[Buf_rms_idx];
			Buf_Irms[Buf_rms_idx] = (Irms / 100.0);
			Buf_rms_idx = (Buf_rms_idx + 1) % 100;
			float Pa = Vrms * Irms;
			filter(Vrms_F, Vrms, 0.98);
			filter(Pa_F, Pa, 0.98);
			cosf_F = (P_F / Pa_F);

			halfCycleCount += 1;
			if (halfCycleCount % 100 == 0)
			{
				gpio_set_level(PIN_LED, (halfCycleCount / 50) % 2);
				printf("#s %3u F %.2f biasV/I %4d/%4d Vrms %.2f P %f Prms %f cosf %.3f cmd %.3f\n",
					   num_samples, 1 / (2 * Period_F), adc_biasV, adc_biasI, Vrms_F, P_F, Pa_F, cosf_F,
					   command);
				halfCycleCount = 0;
			}
		}
		else
		{
			printf("Temps d'attente maximum de l'échantillonneur dépassée!\n");
		}
	}
}

void triac_controller_init()
{
	ledc_timer_config_t timer_config = {};
	timer_config.speed_mode = LEDC_HIGH_SPEED_MODE;
	timer_config.timer_num = LEDC_TIMER_0;
	timer_config.duty_resolution = LEDC_TIMER_10_BIT;
	timer_config.freq_hz = PWM_FREQUENCY;
	timer_config.clk_cfg = LEDC_AUTO_CLK;
	ESP_ERROR_CHECK(ledc_timer_config(&timer_config));

	ledc_channel_config_t led_config = {};
	led_config.gpio_num = PIN_TRIAC;
	led_config.speed_mode = LEDC_HIGH_SPEED_MODE;
	led_config.channel = LEDC_CHANNEL_0;
	led_config.timer_sel = LEDC_TIMER_0;
	led_config.duty = TRIAC_GATE_IMPULSE_CYCLES << 4;
	led_config.intr_type = LEDC_INTR_DISABLE;
	led_config.hpoint = 0;
	led_config.flags.output_invert = 0;
	ESP_ERROR_CHECK(ledc_channel_config(&led_config));

	LEDC.channel_group[0].channel[0].conf0.sig_out_en = 0;
	printf("Timer et canal de contrôle PWM du TRIAC/SSR initialisé.\n");
}

unsigned esp32_adc_calibrate()
{
	const static bool debug = false;
	esp_err_t err;
	gpio_num_t adc_gpio_num, dac_gpio_num;

	err = adc1_pad_get_io_num(ADC_CALI_CHANNEL, &adc_gpio_num);
	ESP_LOGI("eco_pv32", "Calibration: adc1_pad_get_io_num err = %d, gpio = %d", err, adc_gpio_num);
	assert(err == ESP_OK);
	err = dac_pad_get_io_num(DAC_CALI_CHANNEL, &dac_gpio_num);
	ESP_LOGI("eco_pv32", "Calibration: dac_pad_get_io_num err = %d, gpio = %d", err, dac_gpio_num);
	assert(err == ESP_OK);

	unsigned cal_fwd[256];
	unsigned rmsd = 0;

	printf("Calibration canal ADC %d @ GPIO %d via le canal DAC %d @ GPIO %d...", ADC_CALI_CHANNEL, adc_gpio_num, DAC_CALI_CHANNEL, dac_gpio_num);
	fflush(stdout);
	dac_output_enable(DAC_CALI_CHANNEL);
	adc1_config_width(ADC_WIDTH_BIT_12);
	adc1_config_channel_atten(ADC_CALI_CHANNEL, ADC_ATTEN_DB_12);

	uint8_t dac_output = 0;
	int raw_read;
	for (unsigned i = 0; i < 256; ++i)
		cal_fwd[i] = 0;

	do
	{
		dac_output_voltage(DAC_CALI_CHANNEL, dac_output);
		vTaskDelay(10 / portTICK_PERIOD_MS);

		for (unsigned j = 0; j < 64; ++j)
		{
			int retry = 1000;
			do
			{
				raw_read = adc1_get_raw(ADC_CALI_CHANNEL);
			} while (raw_read < 0 && --retry > 0);

			if (raw_read < 0) raw_read = 0;

			cal_fwd[dac_output] += raw_read;
		}
		rmsd += abs((int)dac_output - (int)(cal_fwd[dac_output] / 1024));
		if (dac_output % 16 == 0) {
			esphome::App.feed_wdt();
		}
	} while (++dac_output != 0);

	unsigned x = 0;
	ADC_calibration[0] = 0;
	ADC_calibration[257] = 256;
	for (unsigned i = 0; i < 256; ++i)
	{
		while (cal_fwd[x] <= 1024 * i)
		{
			ADC_calibration[i + 1] = x++;
		}
		x = ((x > 0) ? x - 1 : 0);
	}

	if (debug)
	{
		printf("(rmsd %u):\n", rmsd);
		for (unsigned i = 0; i < 258; ++i)
			printf("Sortie ADC %u Corrigée %d\n", i, ADC_calibration[i]);
	}
	dac_output_disable(DAC_CALI_CHANNEL);
	printf("Ok.\n");
	return rmsd;
}

void set_DAC_biases()
{
	esp_err_t err;
	gpio_num_t dac_gpio_num;

	err = dac_pad_get_io_num(DAC_BIASV_CHANNEL, &dac_gpio_num);
	ESP_LOGI("eco_pv32", "set_DAC_biases: BIASV err = %d, gpio = %d", err, dac_gpio_num);
	assert(err == ESP_OK);
	dac_output_enable(DAC_BIASV_CHANNEL);
	dac_output_voltage(DAC_BIASV_CHANNEL, 128);

	err = dac_pad_get_io_num(DAC_CALI_CHANNEL, &dac_gpio_num);
	ESP_LOGI("eco_pv32", "set_DAC_biases: CALI err = %d, gpio = %d", err, dac_gpio_num);
	assert(err == ESP_OK);
	dac_output_enable(DAC_CALI_CHANNEL);
	dac_output_voltage(DAC_CALI_CHANNEL, 128);
}

static portMUX_TYPE timer_mux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onTimer(void *arg)
{
	int V, I, phase_corr;
	bool sign;
	static unsigned lsumV = 0;
	static unsigned lsumI = 0;
	static unsigned full_cycle_sample_size = 0;
	static unsigned half_cycle_sample_size = 0;
	static int prev_I = 0;

	portENTER_CRITICAL_ISR(&timer_mux);
	timer_group_clr_intr_status_in_isr(TIMER_GROUP_1, TIMER_1);

	SENS.sar_meas_start1.sar1_en_pad = (1 << ADC_AMPS_CHANNEL);
	int timeout = 1000;
	while (SENS.sar_slave_addr1.meas_status != 0 && --timeout > 0)
		;
	SENS.sar_meas_start1.meas1_start_sar = 0;
	SENS.sar_meas_start1.meas1_start_sar = 1;
	timeout = 1000;
	while (SENS.sar_meas_start1.meas1_done_sar == 0 && --timeout > 0)
		;
	I = recalibrate(SENS.sar_meas_start1.meas1_data_sar);

	SENS.sar_meas_start1.sar1_en_pad = (1 << ADC_VOLT_CHANNEL);
	timeout = 1000;
	while (SENS.sar_slave_addr1.meas_status != 0 && --timeout > 0)
		;
	SENS.sar_meas_start1.meas1_start_sar = 0;
	SENS.sar_meas_start1.meas1_start_sar = 1;
	timeout = 1000;
	while (SENS.sar_meas_start1.meas1_done_sar == 0 && --timeout > 0)
		;
	V = recalibrate(SENS.sar_meas_start1.meas1_data_sar);

	if (full_cycle_sample_size >= SAMPLES_PER_CYCLE)
	{
		adc_biasI = ((lsumI / full_cycle_sample_size) + (63 * (unsigned)adc_biasI)) / 64;
		adc_biasV = ((lsumV / full_cycle_sample_size) + (63 * (unsigned)adc_biasV)) / 64;
		full_cycle_sample_size = 0;
		lsumI = 0;
		lsumV = 0;
	}

	lsumV += V;
	lsumI += I;
	++full_cycle_sample_size;

	sign = (V > adc_biasV);
	phase_corr = ((I - prev_I) * phase_shift_cor) / 64;
	prev_I = I;
	V = (V - adc_biasV);
	I = (I - adc_biasI) + phase_corr;

	if ((half_cycle_sample_size < (SAMPLES_PER_CYCLE * 95) / 200) || (prev_sign == sign))
	{
		gsumI2[prev_sign] += I * I;
		gsumV2[prev_sign] += V * V;
		gsumP[prev_sign] += V * I;
		++half_cycle_sample_size;
	}
	else
	{
		lastzc_time = zc_time;
		zc_time = REG_READ(FRC_TIMER_COUNT_REG(1));

		if (sign)
			REG_WRITE(GPIO_OUT_W1TS_REG, (1 << PIN_ZC));
		else
			REG_WRITE(GPIO_OUT_W1TC_REG, (1 << PIN_ZC));

		gsumI2[sign] = I * I;
		gsumV2[sign] = V * V;
		gsumP[sign] = I * V;
		prev_sign = sign;

		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xTaskNotifyFromISR(SA, half_cycle_sample_size, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
		half_cycle_sample_size = 1;

		if (xHigherPriorityTaskWoken)
		{
			portYIELD_FROM_ISR();
		}
	}
	timer_group_enable_alarm_in_isr(TIMER_GROUP_1, TIMER_1);
	portEXIT_CRITICAL_ISR(&timer_mux);
}

void sampling_isr_init()
{
	REG_WRITE(FRC_TIMER_LOAD_REG(1), 0);
	REG_WRITE(FRC_TIMER_CTRL_REG(1), FRC_TIMER_PRESCALER_1 | FRC_TIMER_ENABLE);

	timer_config_t config = {};
	config.divider = TIMER_DIVIDER;
	config.counter_dir = TIMER_COUNT_UP;
	config.counter_en = TIMER_PAUSE;
	config.alarm_en = TIMER_ALARM_EN;
	config.intr_type = TIMER_INTR_LEVEL;
	config.auto_reload = TIMER_AUTORELOAD_EN;
#ifdef TIMER_GROUP_SUPPORTS_XTAL_CLOCK
	config.clk_src = TIMER_SRC_CLK_APB;
#endif
	ESP_LOGI("eco_pv32", "sampling_isr_init: timer_init start");
	timer_init(TIMER_GROUP_1, TIMER_1, &config);
	timer_set_counter_value(TIMER_GROUP_1, TIMER_1, 0x00000000ULL);
	timer_set_alarm_value(TIMER_GROUP_1, TIMER_1, alarm_value);
	timer_enable_intr(TIMER_GROUP_1, TIMER_1);
	ESP_LOGI("eco_pv32", "sampling_isr_init: registering ISR");
	esp_err_t isr_err = timer_isr_register(TIMER_GROUP_1, TIMER_1, onTimer, NULL, 3 | ESP_INTR_FLAG_IRAM, NULL);
	ESP_LOGI("eco_pv32", "sampling_isr_init: timer_isr_register err = %d", isr_err);

	// No blocking waiting for zero-crossing during startup to prevent watchdog reset
	lastzc_time = REG_READ(FRC_TIMER_COUNT_REG(1));
	timer_start(TIMER_GROUP_1, TIMER_1);
}

float estimate_load()
{
	int save_cm = command_mode;
	command_mode = -1;
	delay(5000);
	float P_load = -P_F;
	float V1 = Vrms_F;

	command_mode = 1;
	delay(5000);
	P_load += P_F;
	command = 0;
	command_mode = save_cm;
	return V1 * Vrms_F / P_load;
}

//-------------------------------------------------------------
//      ESPHome Custom Component Definition
// ------------------------------------------------------------
class EcoPV32Component : public esphome::Component {
 public:
  static EcoPV32Component *&instance() {
    static EcoPV32Component *inst = nullptr;
    return inst;
  }

  esphome::sensor::Sensor *p_f_sensor = new esphome::sensor::Sensor();
  esphome::sensor::Sensor *vrms_f_sensor = new esphome::sensor::Sensor();
  esphome::sensor::Sensor *cosf_f_sensor = new esphome::sensor::Sensor();
  esphome::sensor::Sensor *command_sensor = new esphome::sensor::Sensor();

  float set_point_input = 0.0;
  int command_mode_input = 2; // Default to PWM mode
  float command_input = 0.0;

  void setup() override {
	instance() = this;
	ESP_LOGI("eco_pv32", "setup: PIN_TRIAC config start");
	// Configuration pin TRIAC: sortie à 0 (OFF)
	esp_rom_gpio_pad_select_gpio(PIN_TRIAC);
	gpio_set_direction(PIN_TRIAC, GPIO_MODE_OUTPUT);
	gpio_set_level(PIN_TRIAC, 0);

	ESP_LOGI("eco_pv32", "setup: PIN_LED config start");
	// Activation pin LED
	esp_rom_gpio_pad_select_gpio(PIN_LED);
	gpio_set_direction(PIN_LED, GPIO_MODE_OUTPUT);

	ESP_LOGI("eco_pv32", "setup: PIN_ZC config start");
	// Activation pin ZC
	assert(PIN_ZC < 32);
	esp_rom_gpio_pad_select_gpio(PIN_ZC);
	gpio_set_direction(PIN_ZC, GPIO_MODE_OUTPUT);

	ESP_LOGI("eco_pv32", "setup: APB clock config start");
	// Calibration timer FRC2
	apb_freq = rtc_clk_apb_freq_get();

	int cal_attempts = 0;
	unsigned rmsd = 99999;
	while (true) {
		rmsd = esp32_adc_calibrate();
		ESP_LOGI("eco_pv32", "setup: ADC calibration attempt %d completed (rmsd = %u)", cal_attempts + 1, rmsd);
		if (rmsd <= 1750 || ++cal_attempts >= 10) {
			break;
		}
		delay(10);
		esphome::App.feed_wdt();
	}

	ESP_LOGI("eco_pv32", "setup: compute e2d table start");
	// Calculer la table
	compute_e2d_table();

	ESP_LOGI("eco_pv32", "setup: set DAC biases start");
	// Fixer les DAC pour les biais
	set_DAC_biases();

	ESP_LOGI("eco_pv32", "setup: creating sample_analyzer task");
	// Tâche d'analyse sur le core 1
	xTaskCreatePinnedToCore(sample_analyzer, "SA", 2 * 1024, NULL, 3, &SA, 1);

	ESP_LOGI("eco_pv32", "setup: sampling_isr_init start");
	// Démarrer le timer interruptions
	sampling_isr_init();

	ESP_LOGI("eco_pv32", "setup: triac_controller_init start");
	// Préparer le générateur d'impulsions TRIAC
	triac_controller_init();

	ESP_LOGI("eco_pv32", "setup: linky task start");
	// Tâche Linky UART sur le core 0
	linky_init_uart();
	xTaskCreatePinnedToCore(linky_event_task, "linky", 2 * 1024, NULL, 2, &Linky, 0);
	ESP_LOGI("eco_pv32", "setup: complete");

	// Estimation de charge (Disabled to prevent blocking boot and triggering watchdog rollback)
	// R_est = estimate_load();
	// ESP_LOGI("eco_pv32", "Estimated load resistance: %f Ohm", R_est);
  }

  void loop() override {
	// Mettre à jour les variables globales depuis les commandes ESPHome
	set_point = set_point_input;
	command_mode = command_mode_input;
	if (command_mode == 2) {
		command = command_input;
	}

	// Publication périodique des mesures
	static uint32_t last_update = 0;
	uint32_t now = millis();
	if (now - last_update > 1000) {
		last_update = now;
		p_f_sensor->publish_state(P_F);
		vrms_f_sensor->publish_state(Vrms_F);
		cosf_f_sensor->publish_state(cosf_F);
		command_sensor->publish_state(command * 100.0); // Publish as percentage 0-100%
	}
  }
};
