# coheteria-altimeter

# Proyecto Astreo: Aviónica de Vuelo

Este repositorio contiene el firmware y los esquemas de hardware para la computadora de vuelo del **Proyecto Astreo**, un desarrollo de cohetería experimental amateur. El proyecto se enmarca dentro de las Prácticas Profesionales Supervisadas (PPS) de la Facultad de Ciencias Exactas, Físicas y Naturales (FCEFyN) de la Universidad Nacional de Córdoba.

El objetivo principal de esta placa es estimar la cinemática del cohete en tiempo real, detectar el momento exacto del apogeo y comandar el despliegue del paracaídas de forma segura.

## Hardware principal
* **Cerebro:** STM32F103C8T6 (BluePill)
* **Sensores:** Barómetro BMP280 e IMU MPU6050 comunicados por I2C.
* **Alimentación:** Batería de 9V regulada a 3.3V mediante un módulo step-down MP1584.

## Estructura del repositorio

```text
  Astreo-Avionica
 ┣  archivo-KiCad       # Archivos de diseño de la PCB, esquemáticos y gerbers listos para fabricar.
 ┣  bibliotecas         # Librerías y dependencias necesarias para compilar el firmware.
 ┣  codigo-vuelo        # El firmware principal en C/C++ que se sube al cohete.
 ┣  pruebas-BMP280      # Scripts aislados que usamos para caracterizar y validar el barómetro.
 ┗  pruebas-MOSFET      # Códigos de prueba para ensayar la respuesta transitoria de la etapa de potencia.
