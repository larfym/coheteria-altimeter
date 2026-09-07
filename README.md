# coheteria-altimeter

# Proyecto Astreo: Aviónica de Vuelo

Este repositorio contiene el firmware y los esquemas de hardware para la computadora de vuelo del **Proyecto Astreo**, un desarrollo de cohetería experimental amateur. El proyecto se enmarca dentro de las Prácticas Profesionales Supervisadas (PPS) de la Facultad de Ciencias Exactas, Físicas y Naturales (FCEFyN) de la Universidad Nacional de Córdoba.

El objetivo principal de esta placa es estimar la cinemática del cohete en tiempo real, detectar el momento exacto del apogeo y comandar el despliegue del paracaídas de forma segura.

## Hardware principal
* **Cerebro:** STM32F103C8T6 (BluePill)
* **Sensores:** Barómetro BMP280 e IMU MPU6050 comunicados por I2C.
* **Alimentación:** Batería de 9V regulada a 3.3V mediante un módulo step-down MP1584.

## Funcionamiento de Vuelo
1. Inicialización y Calibración en Rampa
Al encender, el sistema realiza rutinas de seguridad y calibración antes de permitir el vuelo:

Bloqueo de Pirotecnia: Fuerza los pines de los MOSFETs (encargados de disparar las cargas de los paracaídas) a un estado bajo (LOW) de manera explícita y mediante resistencias pulldown de hardware para evitar despliegues accidentales durante el arranque.

Comprobación de Sensores: Inicializa la comunicación I2C con el barómetro (BMP280) y el acelerómetro (MPU6050). Si el acelerómetro falla, el sistema bloquea la ejecución para evitar un vuelo ciego.

Calibración de "Altitud Cero": Toma 30 lecturas barométricas a nivel del suelo durante 1.5 segundos para promediar la presión atmosférica local. A partir de este punto, toda altitud se calcula en base a esta presión inicial.

Indicación Visual: El LED integrado pasa a estar encendido y emite un patrón de latido para indicar que el cohete está armado en la rampa de lanzamiento.

2. Procesamiento de Señales y Filtro Matemático
El código ejecuta un bucle de lectura a 20 Hz. Para evitar que perturbaciones como ondas de choque acústicas o vibraciones supersónicas disparen los paracaídas por error, implementa lógica de filtrado:

Filtro de Ruido Físico: Si el barómetro calcula un cambio de presión que implicaría que el cohete se movió a una velocidad superior a la del sonido (343 m/s) en un solo instante, descarta la lectura como anómala.

Filtro Alpha-Beta: Aplica un algoritmo predictivo para suavizar las estimaciones de altitud y velocidad vertical. Calcula dónde debería estar el cohete basándose en su inercia previa (h_pred) y lo ajusta usando los datos frescos de los sensores, otorgando un 45% de peso a la nueva altitud y un 5% a la corrección de velocidad.

3. Detección de Fases de Vuelo y Doble Despliegue
La máquina de estados avanza según las fuerzas físicas detectadas, sin depender únicamente de un cronómetro:

Despegue: Requiere una aceleración superior a 0.35G por encima de la gravedad normal o superar 1.5 metros de altitud durante al menos 0.5 segundos continuos (10 lecturas). Esto evita falsos positivos por manipulación o ráfagas de viento. Una vez detectado, inicia la grabación en la caja negra y apaga el LED para ahorrar batería.

Apogeo (Paracaídas Secundario / Drogue): Se encarga de separar el cohete en su punto más alto para detener el vuelo balístico. El código cuenta con triple redundancia para garantizar la apertura:

Barométrica: Superó los 5 metros de altitud y la velocidad matemática es 0 o negativa (empezó a caer).

Inercial: Ya pasaron 3 segundos de empuje de motor, superó los 15 metros y detecta una inclinación lateral brusca.

Temporizador Absoluto: Disparo forzado a los 15 segundos tras el despegue como último recurso.

Despliegue Principal: Una vez superado el apogeo y cayendo bajo el paracaídas de frenado, la computadora espera a que la altitud descienda por debajo de los 250 metros para disparar la carga principal y asegurar un aterrizaje suave cerca de la zona de lanzamiento.

Aterrizaje: Cuando la altitud estimada regresa por debajo de los 3 metros habiendo superado el apogeo, el vuelo se marca como finalizado.

4. Sistema de Registro
El guardado de datos está dividido en dos niveles de retención para garantizar la supervivencia de la información ante fallos de batería en el impacto:

Registro en RAM: Durante el vuelo, guarda hasta 2000 pares de datos continuos (altitud y tiempo en milisegundos) en matrices temporales de alta velocidad.

Memoria Flash (Datos Persistentes): Al detectar el aterrizaje, extrae un resumen analítico del vuelo (altitudes máximas, presiones exactas, velocidades promedio y aceleración máxima en fuerzas G) y lo inyecta en el sector 0x0800F800 de la memoria Flash interna del microcontrolador. Estos 8 valores sobreviven aunque la placa se apague por completo.

5. Telemetría y Descarga Serial
Tras aterrizar, el LED emite un parpadeo constante y rápido para facilitar la localización visual. Al conectarlo a una PC o módulo Bluetooth, responde a dos comandos a través del puerto Serial/UART:

Comando F o f: Imprime exclusivamente los 8 parámetros estadísticos clave alojados de manera persistente en la memoria Flash.

Comando S o s: Despliega el reporte analítico completo y vuelca todo el arreglo RAM almacenado en formato CSV (Tiempo, Altitud). Este formato está diseñado para ser copiado directamente a Excel u otras herramientas de graficación para el análisis post-vuelo.

6. Protecciones Electrónicas Incorporadas
El firmware incluye control térmico sobre los periféricos de recuperación. Una vez que se ordena activar la pirotecnia de cualquier paracaídas, el microcontrolador corta la corriente de los MOSFETs exactamente 2 segundos después. Esto evita que las baterías de polímero de litio se cortocircuiten o que los transistores se quemen por esfuerzo continuo una vez que la carga ya explotó.

## Estructura del repositorio

```text
  Astreo-Avionica
 ┣  archivo-KiCad       # Archivos de diseño de la PCB, esquemáticos y gerbers listos para fabricar.
 ┣  bibliotecas         # Librerías y dependencias necesarias para compilar el firmware.
 ┣  codigo-vuelo        # El firmware principal en C/C++ que se sube al cohete.
 ┣  pruebas-BMP280      # Scripts aislados que usamos para caracterizar y validar el barómetro.
 ┗  pruebas-MOSFET      # Códigos de prueba para ensayar la respuesta transitoria de la etapa de potencia.
