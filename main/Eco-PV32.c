//-------------------------------------------------------------
// Informations intéressantes
//-------------------------------------------------------------
// TIC Linky: voir http://hallard.me/demystifier-la-teleinfo/
// un optocoupleur SFH620A ou mieux  LTV-814. Ayant un seul client
// TIC connecté sur Linky, j'ai mis une résistance de 1.2kΩ pour
// avoir un signal série clair. La pull-up est inutile (configurée
// en pull-up en interne pour le port série).
// Voir aussi https://github.com/neomilium/teleinfuse ou
//            https://github.com/hallard/teleinfo/blob/master/teleinfo.c

// Contrôle de la charge: TRIAC/SSR ou IGBT/MOSFET.
// Pour le moment, j'utilise en SSR en contrôle de phase mais il serait possible
// d'utiliser du PWM avec un IGBT/MOSFET (filtrage plus facile peut-être).
// Nécessité d'une charge 100% resistive et 2 MOSFETs/IGBP anti-series (pas de
// freewheeling ou de snubber). Voir par exemple
// See eg. www.eevblog.com/forum/projects/igbt-dimmer-pwm-ac-power-control-for-an-immersion-heater
// qui utilise un optocoupleur H11L1 , un driver de MOSFET ICL7667 et on pourrait compléter
// avec 2 IRG4BC30FD IGBT (ou des MOSFET similaires: SIHA22N60AE-GE3 ou IPA60R125C6 ou
// mieux FDL100N50F). FIltrage HA32L-20A or CW4L2-20A-T (passe-bas fréquence inconnue).
// Voir filtres ici https://www.astuces-pratiques.fr/electronique/le-filtre-secteur,
// ou Shaffner https://www.st.com/content/ccc/resource/technical/document/application_note/89/e9/dd/39/ff/74/4a/55/CD00091944.pdf/files/CD00091944.pdf/jcr:content/translations/en.CD00091944.pdf
// La nécessité d'une tension de 15V typ pour la gachette est pénible, mais tirable du secteur (capacité).
// Ou utiliser un redresseur => un seul MOSFET suffirait.

// Simplifier l'usage et le hardware nécessaire;
// - éviter l'échantillonage: utiliser Linky seulement ?
// - détection ZC: utiliser une alim secteur avec une capacité et une LED/zener ou optocoupleur.
// Voir xlyric.
// - Nécessaires: V (linky ou ADC+transfo), P (linky ou ADC+ICT), ZC (ZC ou ADC/transfo).
// Si 2 sources, calibration automatique.

// => Envoi des info en Wifi sur site Web local ou sur eMonCMS ou similaire.
// => Un serveur web server sur le core 0
// => Un pont "serial IP" pour Tic linky
// => Un if RS485 pour l'énergimètre de l'onduleur ou sur le port 502 de l'onduleur
// => Faire un circuit imprimé sur Kicad
// => une if MQTT pour contrôler des charges et connaître leur état

#include <stdio.h>
#include <math.h>
#include <string.h>

#include "sdkconfig.h"
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

#include "esp_timer.h"
#include <esp_task_wdt.h>

#include <soc/sens_reg.h>
#include <soc/sens_struct.h>
#include <soc/ledc_struct.h>

#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "mqtt_client.h"

#ifdef CONFIG_IDF_TARGET_ESP32
#define CHIP_NAME "ESP32"
#endif

#ifdef CONFIG_IDF_TARGET_ESP32S2BETA
#define CHIP_NAME "ESP32-S2 Beta"
#endif

#define WIFI_MAXIMUM_RETRY (5)

#define PIN_LED (2)	  // pin de la LED bleue sur l'ESP32
#define PIN_TRIAC (5) // pin qui contrôle la gachette du TRIAC/SSR
#define PIN_ZC (18)	  // pin qui montre la détection de ZC. Doit être < 32 (accès direct HW)

#define SAMPLES_PER_CYCLE (200) // nombre de paires d'échantillons (I,V) par cycle.
#define ADC_BITS (12)
#define MAX_ADC_OUTPUT ((1 << ADC_BITS) - 1)
#define ADC_OFFSET (1 << (ADC_BITS - 1))
#define ADC_VOLT_CHANNEL ADC1_CHANNEL_0							  // PIN 36
#define ADC_AMPS_CHANNEL ADC1_CHANNEL_3							  // PIN 39
#define ADC_CALI_CHANNEL ADC1_CHANNEL_5							  // PIN 33
#define DAC_CALI_CHANNEL DAC_CHANNEL_1							  // PIN 25
#define DAC_BIASV_CHANNEL DAC_CHANNEL_2							  // PIN 26
#define TIMER_DIVIDER (2)										  // Division pour le timer échantillonage (80 / 2 = 40 MHz)
#define TIMER_SCALE_SEC (TIMER_BASE_CLK / TIMER_DIVIDER)		  // convertir compteur en secondes
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
// Volts: j'utilise 1.8V d'un transfor récupéré sur un vieux détecteur EDF Tempo.
// Sinon, utiliser un trabsfo 5V comme EcoPV (Marque Hans).
// Un mV mesuré est obtenu après transfo + pont diviseur. Le transfo devise par 230/1.8
// soit 127.66666, délivrant 5.09116882454 V crête à crête). Pour ramener un max de 250V
// en entrée à 3.3 V crpête à crête, il faut diviser par plus de (250/230)*2*1.8*sqrt(2)/3.3
// = 1.68. On va diviser par 2 avec un pont 1kΩ + 1kΩ (pertes de 0.9 mA sous 1.8V = 1.7mW).
#define TRANSFORMERATIO (230.0 / 2.07)
#define VDIVIDERATIO (2.0)
#define VREF (3.3)
#define BIT2VOLTMULTIPLIER ((VREF / (1 << ADC_BITS)) * TRANSFORMERATIO * VDIVIDERATIO)
// Amps  (channel 0): le voltage mesuré doit faire moins que 3.3V crête à crête. Avec un
// courant de moins de 50A (11kW) et un PZCT 1:1000 ICT (100A), on a max 50mA RMSdonc
// 141.421356237mA crête à crête. La charge (burden) doit être inférieure à 3300/141.42
// soit 23.3347475605. Donc 22 ou 18Ω. A 50A pour 18Ω, cela génère un perte de 45mW. Une
// résistance 1/4 W 1% suffit. J'ai mis une 20Ω.
#define BURDENRESISTOR 20.2
#define ICTRATIO 1000
#define BIT2AMPSMULTIPLIER (((VREF / (1 << ADC_BITS)) / BURDENRESISTOR) * ICTRATIO)
//-------------------------------------------------------------
//      Variables globales TODO: faire des classes C++
// ------------------------------------------------------------
// -- Wifi --
static EventGroupHandle_t s_wifi_event_group;
const int WIFI_CONNECTED_BIT = BIT0;
static const char *TAG = "wifi station";
static int s_retry_num = 0;

// -- Linky UART --
QueueHandle_t uart_queue;
TaskHandle_t Linky; // Handle vers le buffer d' l'UART Linky

// -- Échantillonage --
unsigned short ADC_calibration[258];   // Données de calibration pour le convertisseur AD
TaskHandle_t SA;					   // Handle vers la tâche d'analyse des échantillons
unsigned short adc_biasV = ADC_OFFSET; // Offset pour le canal des tensions
unsigned short adc_biasI = ADC_OFFSET; // Offset pour le canal des courants
short phase_shift_cor = 2;			   // Correction de phase
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

// -- Contrôle du TRIAC --
float energy2delay[101];
float set_point = 0.0;		// Puissance active cible
const float PI_gain = 10.0; // Gain du contrôleur PI
const float PI_tau = 1.0;	// Délai du contrôleur PI

//-------------------------------------------------------------
//      Sous-routines WiFi
// ------------------------------------------------------------
// gestion des évènement
static void event_handler(void *arg, esp_event_base_t event_base,
						  int32_t event_id, void *event_data)
{
	if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
	{
		esp_wifi_connect();
	}
	else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
	{
		if (s_retry_num < WIFI_MAXIMUM_RETRY)
		{
			esp_wifi_connect();
			xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
			s_retry_num++;
			ESP_LOGI(TAG, "retry to connect to the AP");
		}
		ESP_LOGI(TAG, "connect to the AP fail");
	}
	else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
	{
		ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
		ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
		s_retry_num = 0;
		xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
	}
}
// Initialisation en mode STA: connection au routeur.
// TODO: commencer en mode AP et configurer le routeur
void wifi_init_sta(void)
{
	s_wifi_event_group = xEventGroupCreate();

	ESP_ERROR_CHECK(esp_netif_init());

	ESP_ERROR_CHECK(esp_event_loop_create_default());
	esp_netif_create_default_wifi_sta();

	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));

	ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
	ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

	wifi_config_t wifi_config = {
		.sta = {
			.ssid = CONFIG_WIFI_SSID,
			.password = CONFIG_WIFI_PASS},
	};
	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
	ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
	ESP_ERROR_CHECK(esp_wifi_start());

	ESP_LOGI(TAG, "wifi_init_sta finished. Connected to AP %s\n", CONFIG_WIFI_SSID);
}

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
	ESP_ERROR_CHECK(uart_driver_install(LINKY_UART_NUM, LINKY_BUFFER_SIZE, 0, 10, &uart_queue, 0)); // No TX buffer
	uart_enable_pattern_det_baud_intr(LINKY_UART_NUM, 0x0D, PATTERN_CHR_NUM, 9, 0, 0);
	uart_pattern_queue_reset(LINKY_UART_NUM, 20);
}

// Réception des messages, un CR (0x0D) est utilisé comme motif pour détecter les lignes
// d'une trame. TODO: serial over IP, tout expédier.
// Les seuls tags qui semblent intéressants pour l'autocalibration sont:
// Historique:
//   - HCHC/HCHP, EJPHN/EJPHPM, BBRHCJB/BBRHPJB/BBRHCJW/BBRHPJW/BBRHCJR/BBRHPJR
//     qui donnent accès à l'énergie active en Wh sur 9 chiffres
//     => puissance active en dérivant
//   - IINST: courant instantané (mais résolution 1 A sur 3 chiffres)
//   - PAPP: puissance apparente (résolution 1 VA sur 5 chiffres)
//
// Standard:
//   - DATE: date et heure courante (mais NTP fait de même)
//   - EAST: énergie active soutirée en Wh, 9 chiffres
//   - EAIT: énergie active injectée en Wh, 9 chiffres
//   - ERQ1: energie réactive en Wh 9 chiffres (ou ERQ4 ?)
//   - IRMS1: intensité efficace, résolution 1 A sur 3 chiffres
//   - URMS1: tension efficace, résolution 1 V sur 3 chiffres
//   - SINSTS: puissance apparente soutirée en VA sur 5 chiffres
//   - UMOY1: tension moyenne (sur 10 minutes au lieu d'1 sec par défaut)
// La puissance apparente en VA et le tension efficace en V devraient donner une meilleure
// estimation du courant.
static void linky_event_task(void *pvParameters)
{
	uart_event_t event;
	size_t buffered_size;
	uint8_t *dtmp = (uint8_t *)malloc(128); // TODO set define

	for (;;)
	{
		// Attente d'évènement UART
		if (xQueueReceive(uart_queue, (void *)&event, (portTickType)portMAX_DELAY))
		{
			bzero(dtmp, 1024);
			ESP_LOGI(TAG, "Évènement UART[%d] :", LINKY_UART_NUM);
			switch (event.type)
			{
			case UART_DATA:
				ESP_LOGI(TAG, "[Donnée UART]: %d", event.size);
				uart_read_bytes(LINKY_UART_NUM, dtmp, event.size, portMAX_DELAY);
				ESP_LOGI(TAG, "[Évènmt UART]:");
				uart_write_bytes(LINKY_UART_NUM, (const char *)dtmp, event.size);
				break;
			//Event of HW FIFO overflow detected
			case UART_FIFO_OVF:
				ESP_LOGI(TAG, "Overflow FIFO UART");
				uart_flush_input(LINKY_UART_NUM);
				xQueueReset(uart_queue);
				break;
			//Event of UART ring buffer full
			case UART_BUFFER_FULL:
				ESP_LOGI(TAG, "Ring buffer UART plein");
				uart_flush_input(LINKY_UART_NUM);
				xQueueReset(uart_queue);
				break;
			//Event of UART RX break detected
			case UART_BREAK:
				ESP_LOGI(TAG, "Rx break UART");
				break;
			//Event of UART parity check error
			case UART_PARITY_ERR:
				ESP_LOGI(TAG, "Erreur de parité UART");
				break;
			//Event of UART frame error
			case UART_FRAME_ERR:
				ESP_LOGI(TAG, "Errur de frame UART");
				break;
			//UART_PATTERN_DET
			case UART_PATTERN_DET:
				uart_get_buffered_data_len(LINKY_UART_NUM, &buffered_size);
				int pos = uart_pattern_pop_pos(LINKY_UART_NUM);
				ESP_LOGI(TAG, "[PATTERN UART DÉTECTÉ] pos: %d, buffer: %d", pos, buffered_size);
				if (pos == -1)
				{
					// The pattern position queue is full, flush the rx buffer here.
					uart_flush_input(LINKY_UART_NUM);
				}
				else
				{
					uart_read_bytes(LINKY_UART_NUM, dtmp, pos, 100 / portTICK_PERIOD_MS);
					uint8_t pat[PATTERN_CHR_NUM + 1];
					memset(pat, 0, sizeof(pat));
					uart_read_bytes(LINKY_UART_NUM, pat, PATTERN_CHR_NUM, 100 / portTICK_PERIOD_MS);
					ESP_LOGI(TAG, "lecture : %s", dtmp);
					ESP_LOGI(TAG, "pattern : %s", pat);
				}
				break;
			//Others
			default:
				ESP_LOGI(TAG, "Évènement UART type: %d", event.type);
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
//      pour le TRIAC/SSR
// ------------------------------------------------------------
// Sur une charge résistive de résistance R, sous  V volts, la
// puissance est V²/R. Sur un demi-cycle, V = Vpeak.sin(t) donc l'énergie
// sur ce demi-cycle est proportionnelle à l'intégrale de sin²(x)
// sur la période. La primitive de sin²(x) est ½(x-sin(2x)/2)).
inline float psin2(float x)
{
	return (0.5 * (x - (sin(2 * x) / 2)));
}

// On inverse l'intégrale sur un quart de cycle par symétrie: on exploite
// le caractère monotone de la primitive dans une recherche dichotomique
void compute_e2d_table()
{
	double x1 = 0;
	double x2, x3;
	static const double tolerance = 1e-6;
	energy2delay[0] = 0.0;

	printf("Tabulation énergie-délai...");
	fflush(stdout);
	// for every percentage of max energy over a half-cycle
	for (int i = 1; i <= 100; ++i)
	{
		double fx = i * (M_PI / 400);
		x3 = x1;
		// trouver x3 où f esttrop grand
		while (psin2(x3) <= fx)
		{
			x1 = x3;
			x3 += (M_PI / 100);
		}
		// prendre le point milieu
		x2 = (x1 + x3) / 2;
		while (fabs(psin2(x2) - fx) > tolerance)
		{
			// decider de quel intervalle choisir
			if (psin2(x2) <= fx)
				x1 = x2;
			else
				x3 = x2;
			x2 = (x1 + x3) / 2;
		}
		// stocker la fraction de demi-cycle nécessaire pour générer
		// la fraction de l'énergie après l'impulsion TRIAC
		energy2delay[i] = x2 / M_PI;
		x1 = x2;
	}
	printf("Ok.\n");
}

// Utilise la table energy2delay pour calculer la fraction de demi-cycle
// nécessaire pour avoirune fraction de l'énergie d'un demi-cycle.
float energy_fraction_to_delay(float E_frac)
{
	E_frac = ((E_frac < 0.0) ? 0.0 : E_frac);
	E_frac = ((E_frac > 1.0) ? 1.0 : E_frac);

	E_frac = 1.0 - E_frac;

	if (E_frac <= 0.5) // tabulation avec interpolation linéaire
	{
		E_frac *= 2.0; // la table est en fraction de demi-cycle
		float alpha = (100 * E_frac) - floor(100 * E_frac);
		return ((1.0 - alpha) * energy2delay[(int)floor(E_frac * 100)] + alpha * energy2delay[(int)ceil(E_frac * 100)]);
	}
	else // calculer ce qui ne doit PAS être envoyé.
	{
		E_frac = (1.0 - E_frac) * 2.0;
		float alpha = (100 * E_frac) - floor(100 * E_frac);
		return 1.0 - ((1.0 - alpha) * energy2delay[(int)floor(E_frac * 100)] + alpha * energy2delay[(int)ceil(E_frac * 100)]);
	}
}
//-------------------------------------------------------------
//      Gestion du TRIAC/SSR. Inspiré de
// https://github.com/masoncj/esp32-dimmer/blob/master/main/main.c
// ------------------------------------------------------------
void set_control(float command)
{
	// si la commande est très basse, on éteint le PWM
	if (command <= 0.01)
	{
		LEDC.channel_group[0].channel[0].conf0.sig_out_en = 0;
	} // sinon, on l'alluime dans la durée
	else if (command >= 0.99)
	{
		LEDC.channel_group[0].channel[0].duty.duty = (1 << (PWM_DUTY_BIT_DEPTH + 3)) - 1;
		LEDC.channel_group[0].channel[0].hpoint.hpoint = 0;
		LEDC.channel_group[0].channel[0].conf0.sig_out_en = 1;
		LEDC.channel_group[0].channel[0].conf1.duty_start = 1;
	}
	else // sinon, on ajuste la position de l'impulsion
	{
		LEDC.channel_group[0].channel[0].duty.duty = TRIAC_GATE_IMPULSE_CYCLES << 4;
		LEDC.channel_group[0].channel[0].hpoint.hpoint = (unsigned)(energy_fraction_to_delay(command) * (1 << PWM_DUTY_BIT_DEPTH));
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
	static float Period_F = 10e-3; // durée d'un demi-cycle attendue 10ms
	static float Vrms_F = 230.0;
	static float Pa_F = 0.0;
	static float P_F = 0.0;
	static float cosf_F = 1.0;
	static float integral = 0.0;
	float command = 0.0;
	float error;

	for (;;)
	{
		// On attend que l'ISR d'échantillonnage nous dise d'y aller
		xResult = xTaskNotifyWait(pdFALSE, ULONG_MAX, &ulNotifiedValue, portMAX_DELAY);
		if (xResult == pdPASS)
		{
			// On remet le compteur de PWM à zéro (ZC)
			LEDC.timer_group[0].timer[0].conf.rst = 1;
			LEDC.timer_group[0].timer[0].conf.rst = 0;

			// calcul puissance, cible et commande du contrôleur PI
			unsigned short num_samples = (unsigned short)ulNotifiedValue;
			float P = (float)gsumP[1 - prev_sign] / num_samples * (BIT2VOLTMULTIPLIER * BIT2AMPSMULTIPLIER);
			error = set_point - P;
			integral += error;
			command = (PI_gain * error) + ((PI_gain / PI_tau) * integral);

			if (command <= 0.0)
			{
				command = 0.0;
				integral -= error; // ne pas intégrer, impossible de faire plus
			}
			else if (command >= 1.0)
			{
				command = 1.0;
				integral -= error; // ne pas intégrer, impossible de faire plus
			}
			set_control(command);

			// Caclcul fréquence (période)
			float delta_zc = zc_time - lastzc_time;
			filter(Period_F, delta_zc / apb_freq, 0.99);

			// Calcul tension, courant et puissance efficace/apparente et cos(phi)
			float Vrms = sqrt(((float)gsumV2[1 - prev_sign] / num_samples) * BIT2VOLTMULTIPLIER * BIT2VOLTMULTIPLIER);
			One_sec_Vrms -= Buf_Vrms[Buf_rms_idx];
			Buf_Vrms[Buf_rms_idx] = (Vrms / 100.0);
			float Irms = sqrt(((float)gsumI2[1 - prev_sign] / num_samples) * BIT2AMPSMULTIPLIER * BIT2AMPSMULTIPLIER);
			One_sec_Irms -= Buf_Irms[Buf_rms_idx];
			Buf_Irms[Buf_rms_idx] = (Irms / 100.0);
			Buf_rms_idx = (Buf_rms_idx + 1) % 100;
			float Pa = Vrms * Irms;
			float cos_phi = (P / Pa);
			//float sin_phi = sqrt(1 - (cos_phi * cos_phi));
			filter(Vrms_F, Vrms, 0.99);
			filter(Pa_F, Pa, 0.99);
			filter(P_F, P, 0.99);
			filter(cosf_F, cos_phi, 0.99);

			if (++halfCycleCount % 50 == 0)
			{
				/*
				// gestion type PLL (à tester)
				if (num_samples < (SAMPLES_PER_CYCLE / 2))
				{
					alarm_value = alarm_value - 1;
					timer_set_alarm_value(TIMER_GROUP_1, TIMER_1, alarm_value);
				}
				else if (num_samples > (SAMPLES_PER_CYCLE / 2))
				{
					alarm_value = alarm_value + 1;
					timer_set_alarm_value(TIMER_GROUP_1, TIMER_1, alarm_value);
				}
				*/
				// On fait clignoter la LED bleue chaque demi-seconde
				gpio_set_level(PIN_LED, (halfCycleCount / 50) % 2);
				printf("samples %3u freq %.2f adc_biasV/I %4d/%4d Vrms %.2f P %f Prms %f cos(phi) %.3f command %.2f\n", num_samples, 1 / (2 * Period_F), adc_biasV, adc_biasI, Vrms_F, P_F, Pa_F, cosf_F, command);
			}
		}
		else
		{ // ne devrait jamais arriver (ou changer le temps dans xNotifyWait)
			printf("Temps d'attente maximum de l'échantillonneur dépassée!\n");
		}
	}
}

//-------------------------------------------------------------
//      Gestion du TRIAC via un canal du LEDC PWM de l'ESP32
// ------------------------------------------------------------
// initialisation du contrôleur
void triac_controller_init()
{
	// configuration du timer utilisé
	ledc_timer_config_t timer_config = {
		.speed_mode = LEDC_HIGH_SPEED_MODE,
		.timer_num = 0,
		.bit_num = PWM_DUTY_BIT_DEPTH,
		.freq_hz = PWM_FREQUENCY,
	};
	ESP_ERROR_CHECK(ledc_timer_config(&timer_config));

	// configuration du canal du contrôleur utilisé
	ledc_channel_config_t led_config = {
		.gpio_num = PIN_TRIAC,
		.speed_mode = LEDC_HIGH_SPEED_MODE,
		.channel = 0,
		.timer_sel = LEDC_TIMER_0,
		.duty = TRIAC_GATE_IMPULSE_CYCLES << 4,
		.intr_type = LEDC_INTR_DISABLE,
	};

	ESP_ERROR_CHECK(ledc_channel_config(&led_config));

	LEDC.channel_group[0].channel[0].conf0.sig_out_en = 0;
	printf("Timer et canal de contrôle PWM du TRIAC/SSR initialisé.\n");
}

//-------------------------------------------------------------
//      Calibration de l'ADC de l'ESP32
// ------------------------------------------------------------
// Comme le canal DAC est aussi utilisé comme référence point-milieu
// pour le transfo ICT, il est préférable de lancer la procédure avec
// le transfo ICT déconnecté.

// Utilise un des DAC 8 bits de l'ESP32 et un canal ADC pour
// comparer la mesure ADC à la sortie du DAC attendue.
// NB: 1) la linéarité du DAC a été vérifiée avec un ADC MCP3204.
//     2) les ADC de l'ESP32 sont clairement non linéaires.
void esp32_adc_calibrate()
{
	const static bool debug = false;
	esp_err_t err;

	gpio_num_t adc_gpio_num, dac_gpio_num;
	err = adc1_pad_get_io_num(ADC_CALI_CHANNEL, &adc_gpio_num);
	assert(err == ESP_OK);
	err = dac_pad_get_io_num(DAC_CALI_CHANNEL, &dac_gpio_num);
	assert(err == ESP_OK);

	unsigned cal_fwd[256];

	printf("Calibration canal ADC %d @ GPIO %d via le canal DAC %d @ GPIO %d...", ADC_CALI_CHANNEL, adc_gpio_num, DAC_CALI_CHANNEL, dac_gpio_num);
	fflush(stdout);
	dac_output_enable(DAC_CALI_CHANNEL);
	// utiliser l'échelle 3.3V entière sur 12 bits.
	adc1_config_width(ADC_WIDTH_BIT_12);
	adc1_config_channel_atten(ADC_CALI_CHANNEL, ADC_ATTEN_11db);

	uint8_t dac_output = 0;
	int raw_read;
	for (unsigned i = 0; i < 256; ++i)
		cal_fwd[i] = 0;

	do
	{
		dac_output_voltage(DAC_CALI_CHANNEL, dac_output);
		// Moyenner les échantillons
		for (unsigned j = 0; j < 64; ++j)
		{
			raw_read = adc1_get_raw(ADC_CALI_CHANNEL);
			cal_fwd[dac_output] += raw_read;
		}
	} while (++dac_output != 0);

	// Inverser et filtrer
	unsigned x = 0;
	ADC_calibration[0] = 0;
	ADC_calibration[257] = 256;
	for (unsigned i = 0; i < 256; ++i)
	{
		while (cal_fwd[x] <= 1024 * i) // 10 bits: 4 bits de différence de résolution ADC/AC + 6 bits de suréchantillonnage
		{
			ADC_calibration[i + 1] = x++;
		}
		--x;
	}

	if (debug)
	{
		printf("Calibration ADC:\n");
		for (unsigned i = 0; i < 258; ++i)
			printf("Sortie ADC %d Corrigée %d\n", i, ADC_calibration[i]);
		for (unsigned i = 0; i < MAX_ADC_OUTPUT; ++i)
			printf("Sortie ADC %d Corrigée %d\n", i, recalibrate(i));
	}
	dac_output_disable(DAC_CALI_CHANNEL);
	printf("Ok.\n");
}

// Fixer les sorties des DAC au point milieu pour les biais V/I.
void set_DAC_biases()
{
	esp_err_t err;
	gpio_num_t dac_gpio_num;

	err = dac_pad_get_io_num(DAC_BIASV_CHANNEL, &dac_gpio_num);
	assert(err == ESP_OK);
	dac_output_enable(DAC_BIASV_CHANNEL);
	dac_output_voltage(DAC_BIASV_CHANNEL, 128);

	err = dac_pad_get_io_num(DAC_CALI_CHANNEL, &dac_gpio_num);
	assert(err == ESP_OK);
	dac_output_enable(DAC_CALI_CHANNEL);
	dac_output_voltage(DAC_CALI_CHANNEL, 128);
}

//-------------------------------------------------------------
//      Échantillonnage en IRam par interruption timer
//      Utilise des accès direct au hardware pour
//      le timer, les adc et une gpio (pour éviter les appels
//      hors IRAM). La routinr prend environ 38 µS.
// ------------------------------------------------------------
void IRAM_ATTR onTimer()
{
	int V, I, phase_corr;
	bool sign;
	static unsigned lsumV = 0;
	static unsigned lsumI = 0;
	static unsigned full_cycle_sample_size = 0;
	static unsigned half_cycle_sample_size = 0;
	static int prev_I = 0;

	// section critique
	timer_spinlock_take(TIMER_GROUP_1);
	timer_group_clr_intr_status_in_isr(TIMER_GROUP_1, TIMER_1);

	// échantillonnage I
	SENS.sar_meas_start1.sar1_en_pad = (1 << ADC_AMPS_CHANNEL);

	// The following loop takes ~ 1 µS
	while (SENS.sar_slave_addr1.meas_status != 0)
		;

	SENS.sar_meas_start1.meas1_start_sar = 0;
	SENS.sar_meas_start1.meas1_start_sar = 1;

	// The following loop takes ~ 7.5 µS
	while (SENS.sar_meas_start1.meas1_done_sar == 0)
		;

	I = recalibrate(SENS.sar_meas_start1.meas1_data_sar);

	// échantillonnage V
	SENS.sar_meas_start1.sar1_en_pad = (1 << ADC_VOLT_CHANNEL);
	while (SENS.sar_slave_addr1.meas_status != 0)
		;
	SENS.sar_meas_start1.meas1_start_sar = 0;
	SENS.sar_meas_start1.meas1_start_sar = 1;
	while (SENS.sar_meas_start1.meas1_done_sar == 0)
		;
	V = recalibrate(SENS.sar_meas_start1.meas1_data_sar);

	// calcul et filtrage des biais sur un cycle. TODO: utiliser zc+sign
	if (full_cycle_sample_size >= SAMPLES_PER_CYCLE)
	{
		adc_biasI = ((lsumI / full_cycle_sample_size) + (63 * (unsigned)adc_biasI)) / 64;
		adc_biasV = ((lsumV / full_cycle_sample_size) + (63 * (unsigned)adc_biasV)) / 64;
		full_cycle_sample_size = 0;
		lsumI = 0;
		lsumV = 0;
	}

	// Mise à jour des sommes I/V pour calcul biais
	lsumV += V;
	lsumI += I;
	++full_cycle_sample_size;

	// calcul signe, correction de phase et V/I
	sign = (V > adc_biasV);
	phase_corr = ((I - prev_I) * phase_shift_cor) / 64;
	prev_I = I;
	V = (V - adc_biasV);
	I = (I - adc_biasI) + phase_corr;

	if ((half_cycle_sample_size < SAMPLES_PER_CYCLE * 9 / 20) || (prev_sign == sign))
	{ // si < 90% d'un demi-cycle parcouru ou sile signe est inchangé: accumuler
		gsumI2[prev_sign] += I * I;
		gsumV2[prev_sign] += V * V;
		gsumP[prev_sign] += V * I;
		++half_cycle_sample_size;
	}
	else
	{ // sinon, le signe change dans une zone attendue: mesurer les temps de ZC
		lastzc_time = zc_time;
		zc_time = REG_READ(FRC_TIMER_COUNT_REG(1));

		// indiquer le ZC sur la pin de ZC
		if (sign)
			GPIO.out_w1ts = (1 << PIN_ZC);
		else
			GPIO.out_w1tc = (1 << PIN_ZC);

		// remettre à jour les stats de l'autre demi-cycle (sign)
		gsumI2[sign] = I * I;
		gsumV2[sign] = V * V;
		gsumP[sign] = I * V;
		// on a changé de demi-cycle
		prev_sign = sign;

		// Notifier la tâche d'analyse du zero-crossing
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xTaskNotifyFromISR(SA, half_cycle_sample_size, eSetValueWithOverwrite, &xHigherPriorityTaskWoken);
		// redémarrage du compteur
		half_cycle_sample_size = 1;

		if (xHigherPriorityTaskWoken)
		{
			portYIELD_FROM_ISR();
		}
	}
	// Fin section critique
	timer_group_enable_alarm_in_isr(TIMER_GROUP_1, TIMER_1);
	timer_spinlock_give(TIMER_GROUP_1);
}

void sampling_isr_init()
{
	// on suppose que FRC2 est libre (LAC TG0 utilisé comme HR)
	REG_WRITE(FRC_TIMER_LOAD_REG(1), 0);
	REG_WRITE(FRC_TIMER_CTRL_REG(1), FRC_TIMER_PRESCALER_1 | FRC_TIMER_ENABLE);

	// initialiser timer interruption
	timer_config_t config;
	config.divider = TIMER_DIVIDER;
	config.counter_dir = TIMER_COUNT_UP;
	config.counter_en = TIMER_PAUSE;
	config.alarm_en = TIMER_ALARM_EN;
	config.intr_type = TIMER_INTR_LEVEL;
	config.auto_reload = true;
#ifdef TIMER_GROUP_SUPPORTS_XTAL_CLOCK
	config.clk_src = TIMER_SRC_CLK_APB;
#endif
	timer_init(TIMER_GROUP_1, TIMER_1, &config);
	timer_set_counter_value(TIMER_GROUP_1, TIMER_1, 0x00000000ULL);
	timer_set_alarm_value(TIMER_GROUP_1, TIMER_1, alarm_value);
	timer_enable_intr(TIMER_GROUP_1, TIMER_1);
	timer_isr_register(TIMER_GROUP_1, TIMER_1, onTimer, NULL, 3 | ESP_INTR_FLAG_IRAM, NULL);

	// Démarrage ADC sur un point de zero-crossing.
	// Tant qu'on est en positif, on attend
	while (adc1_get_raw(ADC_VOLT_CHANNEL) > adc_biasV)
		;
	// et on démarre au premier négatif
	while (adc1_get_raw(ADC_VOLT_CHANNEL) <= adc_biasV)
		;
	lastzc_time = REG_READ(FRC_TIMER_COUNT_REG(1));
	timer_start(TIMER_GROUP_1, TIMER_1);
}

//-------------------------------------------------------------
//      MQTT
// ------------------------------------------------------------
static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event)
{
	esp_mqtt_client_handle_t client = event->client;
	int msg_id;
	// your_context_t *context = event->context;
	switch (event->event_id)
	{
	case MQTT_EVENT_CONNECTED:
		ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
		msg_id = esp_mqtt_client_subscribe(client, "gBridge/u81/stat/SonoffAmpli/POWER", 0);
		ESP_LOGI(TAG, "sent subscribe successful, msg_id=%d", msg_id);

		msg_id = esp_mqtt_client_publish(client, "gBridge/u81/cmnd/SonoffAmpli/POWER", "ON", 0, 1, 0);
		ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
		break;
	case MQTT_EVENT_DISCONNECTED:
		ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
		break;

	case MQTT_EVENT_SUBSCRIBED:
		ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
		msg_id = esp_mqtt_client_publish(client, "/topic/qos0", "data", 0, 0, 0);
		ESP_LOGI(TAG, "sent publish successful, msg_id=%d", msg_id);
		break;
	case MQTT_EVENT_UNSUBSCRIBED:
		ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
		break;
	case MQTT_EVENT_PUBLISHED:
		ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
		break;
	case MQTT_EVENT_DATA:
		ESP_LOGI(TAG, "MQTT_EVENT_DATA");
		printf("TOPIC=%.*s\r\n", event->topic_len, event->topic);
		printf("DATA=%.*s\r\n", event->data_len, event->data);
		break;
	case MQTT_EVENT_ERROR:
		ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
		break;
	default:
		ESP_LOGI(TAG, "Other event id:%d", event->event_id);
		break;
	}
	return ESP_OK;
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
	ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);
	mqtt_event_handler_cb(event_data);
}

static void mqtt_app_start(void)
{
	esp_mqtt_client_config_t mqtt_cfg = {
		.uri = CONFIG_BROKER_URL,
	};

	esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
	esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
	esp_mqtt_client_start(client);
}

//-------------------------------------------------------------
//      Main
// ------------------------------------------------------------
void app_main(void)
{
	// Configuration pin TRIAC: sortie à 0 (OFF)
	gpio_pad_select_gpio(PIN_TRIAC);
	gpio_set_direction(PIN_LED, GPIO_MODE_OUTPUT);
	gpio_set_level(PIN_TRIAC, 0);

	// Activation pin LED bleue sur la carte ESP32
	gpio_pad_select_gpio(PIN_LED);
	gpio_set_direction(PIN_LED, GPIO_MODE_OUTPUT);

	// Activation pin zero-crossing
	assert(PIN_ZC < 32); // On utilise l'accès direct au registre GPIO dans l'ISR. Il faut du code dédié pour les pins>=32
	gpio_pad_select_gpio(PIN_ZC);
	gpio_set_direction(PIN_ZC, GPIO_MODE_OUTPUT);

	// Calibration timer FRC2
	apb_freq = rtc_clk_apb_freq_get();

	// Initialiser NVS (WiFi, TODO: autres paramètres. à mettre en sous-routine
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
	{
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

	// calibrer l'ADC
	esp32_adc_calibrate();

	// Calculer la table energy2delay
	compute_e2d_table();

	// se connecter au routeur
	wifi_init_sta();
	vTaskDelay(3000 / portTICK_PERIOD_MS);

	// démarrer l'écoute Linky via l'UART: initialisation et démarrage tâche d'écoute
	linky_init_uart();
	xTaskCreatePinnedToCore(linky_event_task, "linky", 2 * 1024, NULL, 2, &Linky, 0);

	// fixer les DAC pour les biais
	set_DAC_biases();

	// se préparer à analyser les échantillons sur le core 0. TODO: réduire la taille de la pile
	xTaskCreatePinnedToCore(sample_analyzer, "SA", 2 * 1024, NULL, 3, &SA, 1);

	// Démarrer le timer pour interruptions
	sampling_isr_init();

	// préparer le générateur d'impulsions pour le TRIAC/SSR
	triac_controller_init();

	mqtt_app_start();

	for (;;)
		vTaskDelay(50);
}
