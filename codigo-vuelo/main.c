/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Computadora de Vuelo - Doble Despliegue y Registro de Datos
  *                   Este código lee sensores barométricos e inerciales, aplica
  *                   un filtro matemático para estimar altitud y velocidad, detecta 
  *                   el apogeo para desplegar paracaídas, registra los datos 
  *                   en memoria y permite su descarga por puerto serie.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>    // Para el uso de printf
#include <stdlib.h>   // Funciones estándar de C
#include <math.h>     // Operaciones matemáticas (potencias, raíces cuadradas)
#include <stdbool.h>  // Uso de variables booleanas (true/false)
#include "mpu6050.h"  // Librería del acelerómetro/giroscopio
#include "bmp280.h"   // Librería del barómetro (presión y temperatura)
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// --- CONFIGURACIÓN DE PINES E INDICADORES ---
#define LED_PIN GPIO_PIN_13 // Pin del LED indicador integrado en la placa
#define LED_PORT GPIOC      // Puerto del LED indicador

// --- PINES PARA SISTEMA DE RECUPERACIÓN (DOBLE DESPLIEGUE) ---
#define DROGUE_PIN GPIO_PIN_0 // PA0: Pin que activa el mosfet del Paracaídas Secundario (Apogeo)
#define DROGUE_PORT GPIOA
#define MAIN_PIN GPIO_PIN_1   // PA1: Pin que activa el mosfet del Paracaídas Principal (a 250m)
#define MAIN_PORT GPIOA
/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;   // Manejador del bus I2C (comunicación con sensores)
UART_HandleTypeDef huart1; // Manejador del bus UART (comunicación con PC/Bluetooth)

/* USER CODE BEGIN PV */
// --- VARIABLES DE REGISTRO EN RAM (CAJA NEGRA TEMPORAL) ---
#define MAX_SAMPLES 2000  // Límite máximo de muestras a guardar en memoria RAM durante el vuelo

float historial_altitud[MAX_SAMPLES];   // Arreglo para guardar la altitud de cada instante
uint32_t historial_tiempo[MAX_SAMPLES]; // Arreglo para guardar el milisegundo exacto de cada medición

// --- ESTADOS DE VUELO Y TIMERS ---
uint32_t tiempo_despegue = 0;   // Marca de tiempo exacta en la que el cohete abandona la rampa
uint32_t sample_count = 0;      // Contador de muestras guardadas en los arreglos
bool grabando = false;          // Bandera que indica si el cohete está en vuelo y grabando datos
bool prueba_finalizada = false; // Bandera que indica si el vuelo terminó y ya aterrizó
int conteo_despegue = 0;        // Contador de confirmación para evitar falsos despegues

// --- VARIABLES DE SENSORES ---
BMP280_HandleTypedef bmp280;    // Estructura de control para el sensor barométrico
MPU6050_t mpu6050_data;         // Estructura para almacenar los datos brutos del acelerómetro

float temperatura, presion_Pa, humedad, altitud_m; // Variables ambientales leídas del sensor
float P0 = 101325.0f;           // Presión de referencia en el suelo (se calibra al inicio)

// --- FILTRO MATEMÁTICO (ALPHA-BETA) PARA SUAVIZAR EL RUIDO SENSORIAL ---
float h_est = 0;                // Altitud estimada (filtrada)
float v_est = 0;                // Velocidad vertical estimada (filtrada)
uint32_t tiempo_anterior = 0;   // Marca de tiempo de la iteración anterior del bucle
int rechazos_consecutivos = 0;  // Contador de lecturas anómalas descartadas

// --- MEMORIAS DE EVENTOS Y MÁXIMOS ---
float altitud_maxima = 0.0f;      // Altitud máxima alcanzada en todo el vuelo
bool apogeo_detectado = false;    // Bandera que indica si ya pasamos el punto más alto
float altitud_apogeo = 0.0f;      // Altitud exacta en el momento del apogeo

// --- TIMERS Y ESTADOS DE DESPLIEGUE ---
bool main_desplegado = false;     // Bandera para evitar disparar el paracaídas principal dos veces
uint32_t tiempo_drogue = 0;       // Marca de tiempo en la que se activó la carga del apogeo
uint32_t tiempo_main = 0;         // Marca de tiempo en la que se activó la carga principal
uint32_t tiempo_aterrizaje = 0;   // Marca de tiempo del impacto contra el suelo

// --- VARIABLES PARA CÁLCULO DE VELOCIDAD PROMEDIO ---
float velocidad_prom_ascenso = 0.0f;  // Promedio matemático de velocidad subiendo
float velocidad_prom_descenso = 0.0f; // Promedio matemático de velocidad cayendo

// --- VARIABLES DE ESFUERZO ESTRUCTURAL ---
float aceleracion_maxima = 0.0f;      // Máxima fuerza G sufrida por el cohete
uint32_t tiempo_aceleracion_max = 0;  // En qué momento exacto ocurrió la máxima fuerza G
float altitud_aceleracion_max = 0.0f; // A qué altitud ocurrió la máxima fuerza G

// --- VARIABLES ATMOSFÉRICAS CLAVE ---
float presion_altitud_max = 0.0f;     // Presión atmosférica en el punto más alto del vuelo
float presion_apogeo = 0.0f;          // Presión atmosférica en el instante exacto que se detectó apogeo
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
// Prototipos de funciones para manejar la memoria Flash interna del microcontrolador
void Guardar_Datos_En_Flash(float alt_max, float vel_prom_asc, float alt_apog, float ac_max,
                            float pres_alt_max, float pres_apog, float espacio_reservado, float vel_prom_desc);

void Leer_Datos_De_Flash(float* alt_max, float* vel_prom_asc, float* alt_apog, float* ac_max,
                         float* pres_alt_max, float* pres_apog, float* espacio_reservado, float* vel_prom_desc);

// Sobreescritura de la función putchar para que printf() envíe texto a través del puerto serie (UART)
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY); // Envía un carácter por la UART1
    return ch;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

int main(void)
{
  // Inicialización base de los componentes del microcontrolador
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
      // --- 1. INICIALIZAR EL LED INTEGRADO PARA AVISOS VISUALES ---
      __HAL_RCC_GPIOC_CLK_ENABLE(); // Encender el reloj del puerto C
      GPIO_InitTypeDef GPIO_InitStruct = {0};
      GPIO_InitStruct.Pin = LED_PIN;
      GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Modo salida Push-Pull
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
      HAL_GPIO_Init(LED_PORT, &GPIO_InitStruct);

      // --- 2. INICIALIZAR PINES DE PARACAÍDAS (MODO SEGURO) ---
      __HAL_RCC_GPIOA_CLK_ENABLE(); // Encender el reloj del puerto A
      GPIO_InitTypeDef GPIO_InitStruct_Pyro = {0};
      GPIO_InitStruct_Pyro.Pin = DROGUE_PIN | MAIN_PIN;
      GPIO_InitStruct_Pyro.Mode = GPIO_MODE_OUTPUT_PP;
      // CRÍTICO: La resistencia pulldown asegura que los mosfets no se disparen accidentalmente 
      // si el microcontrolador se reinicia o está en proceso de arranque.
      GPIO_InitStruct_Pyro.Pull = GPIO_PULLDOWN; 
      GPIO_InitStruct_Pyro.Speed = GPIO_SPEED_FREQ_LOW;
      HAL_GPIO_Init(GPIOA, &GPIO_InitStruct_Pyro);

      // Forzar explícitamente los mosfets a estado apagado (LOW) al iniciar
      HAL_GPIO_WritePin(DROGUE_PORT, DROGUE_PIN, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(MAIN_PORT, MAIN_PIN, GPIO_PIN_RESET);

      // --- 3. TEST VISUAL DE ARRANQUE ---
      // Hace parpadear el LED 3 veces para indicar que la placa encendió correctamente
      for(int i = 0; i < 3; i++) {
          HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
          HAL_Delay(150);
          HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET); 
          HAL_Delay(150);
      }
      HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);

      // --- 4. INICIALIZACIÓN DE SENSORES ---
      // Configurar y arrancar el sensor de presión BMP280
      bmp280.i2c = &hi2c1;
      bmp280.addr = BMP280_I2C_ADDRESS_0;
      bmp280_params_t params;
      bmp280_init_default_params(&params);
      bmp280_init(&bmp280, &params);

      // Arrancar el acelerómetro MPU6050. Si falla, el sistema se queda colgado apagando el LED.
      if(MPU6050_Init(&hi2c1) == 1) {
          HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
          while(1); // Bucle infinito de error
      }
      HAL_Delay(1000); // Esperar que los sensores se estabilicen

      // --- 5. CALIBRACIÓN DE PRESIÓN BASE (P0) ---
      // Toma 30 lecturas de presión ambiental en rampa y las promedia.
      // Esto establece la "Altitud Cero" relativa al lugar físico del lanzamiento.
      float suma_p = 0;
      for(int i=0; i<30; i++) {
          bmp280_read_float(&bmp280, &temperatura, &presion_Pa, &humedad);
          suma_p += presion_Pa;
          HAL_Delay(50); // Muestrear cada 50ms
      }
      P0 = suma_p / 30.0f; // Guardar el promedio matemático

      // Encender el LED para indicar que el sistema está armado y listo para volar
      HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
      HAL_Delay(500);
  /* USER CODE END 2 */

  /* USER CODE BEGIN WHILE */
      // Bucle principal de ejecución que correrá durante todo el vuelo
      while (1){
          // Obtener el tiempo actual en milisegundos desde que inició el microcontrolador
          uint32_t tiempo_actual = HAL_GetTick();
          // Calcular el delta temporal en segundos desde la última medición
          float delta_t = (tiempo_actual - tiempo_anterior) / 1000.0f;

          // --- PATRONES DE PARPADEO DEL LED (ESTADO DEL SISTEMA) ---
          if (!grabando) {
              if (!prueba_finalizada) {
                  // Estado: En rampa, esperando despegue. Parpadeo corto tipo "latido"
                  if ((tiempo_actual % 1000) < 80) HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
                  else HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
              } else {
                  // Estado: Aterrizado. Parpadeo constante y rápido (Señal para facilitar rescate)
                  if ((tiempo_actual % 500) < 250) HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
                  else HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
              }
          }

          // Ejecutar lecturas de sensores solo cada 50ms (20 veces por segundo) y si no ha aterrizado
          if (delta_t >= 0.05f && !prueba_finalizada) {
              tiempo_anterior = tiempo_actual; // Actualizar la marca de tiempo

              // 1. Leer Barómetro
              if (bmp280_read_float(&bmp280, &temperatura, &presion_Pa, &humedad)) {
                  // Calcular la altitud cruda usando la fórmula barométrica estándar internacional
                  float altitud_cruda = 44330.0f * (1.0f - pow((presion_Pa / P0), 0.1902949f));
                  // Calcular la velocidad bruta como la diferencia de altitud en el tiempo delta
                  float v_bruta = (altitud_cruda - h_est) / delta_t;

                  // Filtro para ignorar picos de presión irreales (como la onda de choque y vibraciones supersónicas)
                  // Se acepta la lectura si implica moverse a menos de la velocidad del sonido (343 m/s)
                  if ((v_bruta > -343.0f && v_bruta < 343.0f) || rechazos_consecutivos > 40) {
                      rechazos_consecutivos = 0; // Reiniciar contador de errores
                      
                      // Filtro Alpha-Beta (Estima el estado real ignorando el ruido del sensor)
                      float alpha_val = 0.45f; // Peso de la lectura actual vs predicción (altitud)
                      float beta_val = 0.05f;  // Peso de la lectura actual vs predicción (velocidad)
                      
                      float h_pred = h_est + (v_est * delta_t); // Predicción de dónde debería estar el cohete
                      float error = altitud_cruda - h_pred;     // Diferencia entre la realidad y la predicción

                      // Actualizar estimaciones oficiales
                      h_est = h_pred + (alpha_val * error); 
                      v_est = v_est + ((beta_val / delta_t) * error); 
                  } else {
                      // Si la lectura era físicamente imposible, la ignoramos y avanzamos con la predicción inercial
                      rechazos_consecutivos++;
                      h_est = h_est + (v_est * delta_t);
                  }
              }

              // 2. Leer Acelerómetro
              MPU6050_Read_Accel(&hi2c1, &mpu6050_data);
              
              // Calcular el vector de magnitud total de aceleración en todos los ejes
              float accel_total = sqrtf((mpu6050_data.Ax * mpu6050_data.Ax) +
                                        (mpu6050_data.Ay * mpu6050_data.Ay) +
                                        (mpu6050_data.Az * mpu6050_data.Az));
              // Calcular cuánto se desvía la aceleración de la gravedad estática (1G)
              float desvio_g = fabsf(accel_total - 1.0f);

              // 3. Registrar los picos máximos del vuelo
              if (grabando || conteo_despegue > 0) {
                  // Récord de Altitud
                  if (h_est > altitud_maxima) {
                      altitud_maxima = h_est;
                      presion_altitud_max = presion_Pa;
                  }

                  // Récord de Esfuerzo Estructural (Aceleración)
                  if (accel_total > aceleracion_maxima) {
                      aceleracion_maxima = accel_total;
                      tiempo_aceleracion_max = tiempo_actual; 
                      altitud_aceleracion_max = h_est;        
                  }
              }

              // --- LÓGICA DE DETECCIÓN DE DESPEGUE ---
              // El sistema no graba hasta que está seguro que despegó
              if (!grabando) {
                  // Si siente una fuerza mayor a 0.35G adicionales, o sube más de 1.5 metros:
                  if (desvio_g > 0.35f || h_est > 1.5f) {
                      conteo_despegue++; 
                  } else if (conteo_despegue > 0) {
                      conteo_despegue--; // Se resta si fue una vibración falsa del viento o manipulación
                  }

                  // Si se cumplen las condiciones 10 veces consecutivas (0.5 segundos reales de empuje continuo)
                  if (conteo_despegue >= 10) {
                      grabando = true; // Inicia estado de vuelo
                      tiempo_despegue = tiempo_actual; // Marca el T=0 del vuelo
                  }
              }

             // --- LÓGICA EN VUELO: APOGEO Y SISTEMAS DE RECUPERACIÓN ---
              if (grabando) {
                  // Apagar LED en vuelo para ahorrar energía de la batería de aviónica
                  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
                  
                  // Calcular fuerza centrífuga/inclinación en los ejes perpendiculares al movimiento
                  // Sirve para saber si el cohete dejó de ir hacia arriba y se acostó horizontalmente
                  float aceleracion_lateral = sqrtf((mpu6050_data.Ax * mpu6050_data.Ax) + 
                                                    (mpu6050_data.Ay * mpu6050_data.Ay));

                  // CONDICIÓN A: Detección Barométrica. El cohete superó los 5m y la velocidad matemática es 0 o negativa (bajando).
                  bool apogeo_barometrico = (h_est > 5.0f && v_est <= 0.0f);

                  // CONDICIÓN B: Detección Inercial de Seguridad. El motor ya se quemó (>3 seg), el cohete está alto, y se inclinó lateralmente de golpe.
                  bool motor_apagado = (tiempo_actual - tiempo_despegue > 3000); 
                  bool apogeo_inercial = (motor_apagado && h_est > 15.0f && aceleracion_lateral > 0.6f);

                  // CONDICIÓN C: Temporizador de Seguridad Absoluta. Si todo falla, dispara sí o sí a los 15 segundos.
                  bool disparo_por_tiempo = (tiempo_actual - tiempo_despegue > 15000); 

                  // EJECUCIÓN DE APOGEO: Si cualquier condición se cumple por primera vez, activar carga.
                  if (!apogeo_detectado && (apogeo_barometrico || apogeo_inercial || disparo_por_tiempo)) {
                      apogeo_detectado = true;
                      altitud_apogeo = h_est; 
                      presion_apogeo = presion_Pa;
                      tiempo_drogue = tiempo_actual;
                      
                      // Encender pirotecnia primaria / Activar actuador
                      HAL_GPIO_WritePin(DROGUE_PORT, DROGUE_PIN, GPIO_PIN_SET); 
                  }
                  
                  // EJECUCIÓN PARACAÍDAS PRINCIPAL: Ya pasó el apogeo y la altitud cayó por debajo de 250 metros.
                  if (apogeo_detectado && !main_desplegado && h_est <= 250.0f) {
                      main_desplegado = true;
                      tiempo_main = tiempo_actual;
                      // Encender pirotecnia secundaria / Activar actuador
                      HAL_GPIO_WritePin(MAIN_PORT, MAIN_PIN, GPIO_PIN_SET);
                  }

                  // PROTECCIÓN TÉRMICA DE LOS MOSFETS: Cortar la corriente a las cargas 2 segundos después de encenderlas
                  // Esto evita que los transistores se quemen o derritan plástico circundante
                  if (apogeo_detectado && (tiempo_actual - tiempo_drogue > 2000)) {
                      HAL_GPIO_WritePin(DROGUE_PORT, DROGUE_PIN, GPIO_PIN_RESET);
                  }
                  if (main_desplegado && (tiempo_actual - tiempo_main > 2000)) {
                      HAL_GPIO_WritePin(MAIN_PORT, MAIN_PIN, GPIO_PIN_RESET);
                  }

                  // --- DETECCIÓN DE ATERRIZAJE NORMAL ---
                  // Ya desplegó y la altitud calculada vuelve a ser casi cero
                  if (apogeo_detectado && h_est < 3.0f) {
                      grabando = false;           // Detener grabación de datos continuos
                      prueba_finalizada = true;   // Pasar al estado de rescate
                      tiempo_aterrizaje = tiempo_actual; // Registrar la hora de toque

                      // Calcular tiempos totales de cada etapa en segundos
                      float tiempo_ascenso_seg = (tiempo_drogue - tiempo_despegue) / 1000.0f;
                      float tiempo_descenso_seg = (tiempo_aterrizaje - tiempo_drogue) / 1000.0f;

                      // Calcular velocidades promedio (V = d/t)
                      if (tiempo_ascenso_seg > 0) velocidad_prom_ascenso = altitud_apogeo / tiempo_ascenso_seg;
                      if (tiempo_descenso_seg > 0) velocidad_prom_descenso = altitud_apogeo / tiempo_descenso_seg;

                      // Guardar bloque de parámetros clave en memoria no volátil para que sobreviva apagados
                      Guardar_Datos_En_Flash(altitud_maxima, velocidad_prom_ascenso, altitud_apogeo, aceleracion_maxima,
                                             presion_altitud_max, presion_apogeo, 0.0f, velocidad_prom_descenso);
                  }

                  // --- GUARDADO DE HISTORIAL CONTINUO EN RAM ---
                  if (sample_count < MAX_SAMPLES) {
                      // Registrar punto de la curva del gráfico
                      historial_altitud[sample_count] = h_est;
                      historial_tiempo[sample_count] = tiempo_actual;
                      sample_count++;
                  } else if (grabando) {
                      // CASO EXTREMO: El arreglo se llenó antes de tocar el suelo.
                      // Detenemos el registro forzosamente para evitar que la memoria colapse,
                      // pero calculamos y guardamos la información recopilada hasta este punto.
                      grabando = false;
                      prueba_finalizada = true;
                      tiempo_aterrizaje = tiempo_actual;

                      float tiempo_ascenso_seg = (tiempo_drogue > 0 ? (tiempo_drogue - tiempo_despegue) / 1000.0f : 0);
                      float tiempo_descenso_seg = (tiempo_drogue > 0 ? (tiempo_aterrizaje - tiempo_drogue) / 1000.0f : 0);

                      if (tiempo_ascenso_seg > 0) velocidad_prom_ascenso = altitud_apogeo / tiempo_ascenso_seg;
                      if (tiempo_descenso_seg > 0) velocidad_prom_descenso = altitud_apogeo / tiempo_descenso_seg;

                      Guardar_Datos_En_Flash(altitud_maxima, velocidad_prom_ascenso, altitud_apogeo, aceleracion_maxima,
                                             presion_altitud_max, presion_apogeo, 0.0f, velocidad_prom_descenso);
                      HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
                  }
              }
          } 

          // --- COMUNICACIÓN SERIE (TELEMETRÍA / DESCARGA DE DATOS) ---
          uint8_t tecla_recibida = 0;
          // Revisa si llegó algún byte por el puerto Serie sin bloquear el código principal
          if (HAL_UART_Receive(&huart1, &tecla_recibida, 1, 0) == HAL_OK) {
              
              // Comando 'F' o 'f': Imprimir los datos guardados en la memoria Flash (sobrevive apagados)
              if (tecla_recibida == 'f' || tecla_recibida == 'F') {
                  // Variables temporales para alojar lo que se extraiga de la memoria
                  float f_alt_max, f_vel_prom_asc, f_alt_apog, f_ac_max;
                  float f_pres_alt_max, f_pres_apog, f_espacio_reservado, f_vel_prom_desc;
                  
                  Leer_Datos_De_Flash(&f_alt_max, &f_vel_prom_asc, &f_alt_apog, &f_ac_max,
                                      &f_pres_alt_max, &f_pres_apog, &f_espacio_reservado, &f_vel_prom_desc);

                  printf("\r\n=====================================\r\n");
                  printf("   REPORTE DE CAJA NEGRA (FLASH)     \r\n");
                  printf("=====================================\r\n");
                  printf("-> Altitud Maxima:        %.2f metros (Presion: %.2f Pa)\r\n", f_alt_max, f_pres_alt_max);
                  printf("-> Altitud en Apogeo:     %.2f metros (Presion: %.2f Pa)\r\n", f_alt_apog, f_pres_apog);
                  printf("-> Vel. Promedio Ascenso: %.2f m/s\r\n", f_vel_prom_asc);
                  printf("-> Vel. Promedio Descenso:%.2f m/s\r\n", f_vel_prom_desc);
                  printf("-> Aceleracion Maxima:    %.2f G\r\n", f_ac_max);
                  printf("=====================================\r\n");
              }

              // Comando 'S' o 's': Volcar reporte completo y tabla CSV con la curva de vuelo entera (solo tras aterrizar)
              if (prueba_finalizada && (tecla_recibida == 's' || tecla_recibida == 'S')) {
                  printf("\r\n=============================\r\n");
                  printf("      REPORTE DE VUELO       \r\n");
                  printf("=============================\r\n");
                  printf("-> Altitud Max. (Absoluta): %.2f metros (Presion: %.2f Pa)\r\n", altitud_maxima, presion_altitud_max);

                  if(apogeo_detectado) {
                      printf("-> Altitud en Apogeo: %.2f metros (Presion: %.2f Pa)\r\n", altitud_apogeo, presion_apogeo);
                  } else {
                      printf("-> Apogeo no detectado\r\n");
                  }

                  printf("\r\n[VELOCIDAD PROMEDIO]\r\n");
                  printf("-> Promedio en Ascenso:  %.2f m/s\r\n", velocidad_prom_ascenso);
                  printf("-> Promedio en Descenso: %.2f m/s\r\n", velocidad_prom_descenso);
                  printf("   - Tiempo de Ascenso:  %.2f seg\r\n", (float)(tiempo_drogue - tiempo_despegue) / 1000.0f);
                  printf("   - Tiempo de Descenso: %.2f seg\r\n", (float)(tiempo_aterrizaje - tiempo_drogue) / 1000.0f);
                  
                  printf("\r\n[ACELERACION]\r\n");
                  printf("-> Aceleracion Maxima: %.2f G\r\n", aceleracion_maxima);
                  printf("   - Ocurrio en (T+): %.2f segundos\r\n", (float)(tiempo_aceleracion_max - tiempo_despegue) / 1000.0f);
                  printf("=====================================\r\n");

                  // Generar formato CSV estándar, ideal para copiar y pegar en Excel / Google Sheets
                  printf("\r\n--- INICIO DESCARGA CSV ---\r\n");
                  printf("TIEMPO(s),ALTITUD(m)\r\n");
                  for (uint32_t i = 0; i < sample_count; i++) {
                      // El tiempo se transmite normalizado, es decir, Despegue = T 0.00
                      printf("%.2f,%.2f\r\n", (float)(historial_tiempo[i] - tiempo_despegue) / 1000.0f, historial_altitud[i]);
                      HAL_Delay(1); // Pequeña pausa para no saturar el buffer USB-Serial
                  }
                  printf("--- FIN DESCARGA CSV ---\r\n");

                  // Parpadeo de LED que confirma que la descarga fue exitosa
                  for(int k=0; k<3; k++) {
                      HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
                      HAL_Delay(200);
                      HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
                      HAL_Delay(200);
                  }
              }
          }
      }
  /* USER CODE END WHILE */
}

/* 
   Configuraciones de periféricos generadas automáticamente por STM32CubeMX.
   Están omitidas en este bloque de texto, son gestionadas directamente por el IDE. 
*/
void SystemClock_Config(void) { /* ... Código autogenerado del reloj ... */ }
static void MX_I2C1_Init(void) { /* ... Código autogenerado I2C ... */ }
static void MX_USART1_UART_Init(void) { /* ... Código autogenerado UART ... */ }
static void MX_GPIO_Init(void) { /* ... Código autogenerado GPIO ... */ }

/* USER CODE BEGIN 4 */
// Definición de la dirección física del chip de memoria donde guardaremos la caja negra (Página 63)
#define FLASH_STORAGE_ADDRESS  0x0800F800 

/**
 * @brief Escribe 8 variables flotantes directamente en la memoria persistente del microcontrolador.
 * Al guardarse aquí, los datos no se borran aunque la placa pierda la batería en el impacto.
 */
void Guardar_Datos_En_Flash(float alt_max, float vel_prom_asc, float alt_apog, float ac_max,
                            float pres_alt_max, float pres_apog, float espacio_reservado, float vel_prom_desc) {
    
    HAL_FLASH_Unlock(); // Desbloquear la escritura de la memoria Flash

    // Configurar el borrado del sector de memoria. Siempre hay que borrar antes de reescribir.
    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PageError = 0;
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.PageAddress = FLASH_STORAGE_ADDRESS;
    EraseInitStruct.NbPages = 1;

    // Si el borrado fue exitoso, proceder a inyectar los datos palabra por palabra
    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) == HAL_OK) {
        
        // Punteros necesarios para engañar al sistema y pasar de formato Flotante a Entero de 32 bits 
        // (La memoria Flash del STM32 graba palabras de 32 bits puros)
        uint32_t* p_alt_max = (uint32_t*)&alt_max;
        uint32_t* p_vel_asc = (uint32_t*)&vel_prom_asc;
        uint32_t* p_alt_apog = (uint32_t*)&alt_apog;
        uint32_t* p_ac_max = (uint32_t*)&ac_max;
        uint32_t* p_pres_alt_max = (uint32_t*)&pres_alt_max;
        uint32_t* p_pres_apog = (uint32_t*)&pres_apog;
        uint32_t* p_reservado = (uint32_t*)&espacio_reservado; // Mantiene el esquema de partición de memoria intacto de 8 bloques.
        uint32_t* p_vel_desc = (uint32_t*)&vel_prom_desc;

        // Grabar cada variable secuencialmente, desplazándose 4 bytes (1 word) cada vez
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_STORAGE_ADDRESS, *p_alt_max);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_STORAGE_ADDRESS + 4, *p_vel_asc);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_STORAGE_ADDRESS + 8, *p_alt_apog);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_STORAGE_ADDRESS + 12, *p_ac_max);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_STORAGE_ADDRESS + 16, *p_pres_alt_max);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_STORAGE_ADDRESS + 20, *p_pres_apog);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_STORAGE_ADDRESS + 24, *p_reservado);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_STORAGE_ADDRESS + 28, *p_vel_desc);
    }
    
    HAL_FLASH_Lock(); // Proteger la memoria para prevenir escrituras accidentales
}

/**
 * @brief Lee directamente las direcciones de la memoria persistente donde se guardaron los datos
 * y los carga en las variables apuntadas.
 */
void Leer_Datos_De_Flash(float* alt_max, float* vel_prom_asc, float* alt_apog, float* ac_max,
                         float* pres_alt_max, float* pres_apog, float* espacio_reservado, float* vel_prom_desc) {
    
    // El formato 'volatile' asegura que el compilador lea directamente el chip 
    // y no use memoria cache o variables almacenadas temporalmente.
    *alt_max = *(volatile float*)(FLASH_STORAGE_ADDRESS);
    *vel_prom_asc = *(volatile float*)(FLASH_STORAGE_ADDRESS + 4);
    *alt_apog = *(volatile float*)(FLASH_STORAGE_ADDRESS + 8);
    *ac_max = *(volatile float*)(FLASH_STORAGE_ADDRESS + 12);
    *pres_alt_max = *(volatile float*)(FLASH_STORAGE_ADDRESS + 16);
    *pres_apog = *(volatile float*)(FLASH_STORAGE_ADDRESS + 20);
    *espacio_reservado = *(volatile float*)(FLASH_STORAGE_ADDRESS + 24);
    *vel_prom_desc = *(volatile float*)(FLASH_STORAGE_ADDRESS + 28);
}
/* USER CODE END 4 */

void Error_Handler(void)
{
  __disable_irq(); // Desactivar interrupciones
  while (1) { }    // Bucle infinito, el micro requiere reinicio manual de emergencia
}
