Flujo del Coidgo Completo

Este código es un sistema **Multitarea (RTOS)** avanzado. A diferencia de un código simple de Arduino que lee línea por línea de arriba a abajo, aquí tienes **tres cerebros** funcionando casi al mismo tiempo.

Para entender el flujo, debemos dividirlo en **Fases** y **Tareas Paralelas**.

---

### FASE 1: El Arranque (`setup`)

Todo comienza aquí. Es lineal y ocurre una sola vez.

1. **Infraestructura:** Se inicia el Puerto Serial, el bus I2C (cables de sensores) y se crea el **Mutex** (la "llave" de seguridad para que los sensores no chiquen).
2. **Radio LoRa:** Se configura el chip de radio primero (SPI). Si falla, te avisa.
3. **Sensores e Interfaz (Zona Protegida):**
* El código toma el Mutex (`xSemaphoreTake`).
* Inicia la Pantalla OLED, el Sensor de Pulso (MAX30102), el BME280 y el MPU9250.
* Suelta el Mutex (`xSemaphoreGive`).


4. **La Bifurcación (Fork):** Aquí el código se divide.
* Lanza la tarea `BiometricTask` al **Núcleo 0**.
* Lanza la tarea `TaskComunicaciones` al **Núcleo 1**.
* El flujo principal cae automáticamente al `loop()` (que corre en el **Núcleo 1**).



---

### FASE 2: La Ejecución Paralela (Las 3 Pistas)

A partir de ahora, imagina tres carriles de una autopista funcionando a la vez. Se comunican entre sí mediante **Variables Globales** (`g_spo2`, `g_estadoGlobal`).

#### Pista 1: El Médico (`BiometricTask` en Core 0)

Esta tarea se dedica exclusivamente a la salud.

1. **Recolección (Bucle Rápido):** Pide la llave (Mutex) y saca datos del sensor MAX30102 hasta llenar una lista de 100 muestras (tarda 1 segundo aprox).
2. **Cálculo Matemático:** Una vez tiene las 100 muestras, ejecuta el algoritmo complejo `maxim_heart_rate...` para calcular BPM y SpO2.
3. **Lectura Ambiental:** Lee presión y humedad del BME280.
4. **Publicación:** Pide la llave de nuevo y actualiza las variables globales `g_heartRate` y `g_spo2` para que los otros núcleos las vean.
5. **Descanso:** Duerme 500ms (`vTaskDelay`) para no sobrecalentar el chip.

#### Pista 2: El Mensajero (`TaskComunicaciones` en Core 1)

Esta tarea es el "operador de radio".

1. **Lectura de Estado:** Mira la variable `g_estadoGlobal`. ¿Dice "CAIDA CONFIRMADA"?
2. **Empaquetado:** Mete los últimos datos de salud y el estado en la cajita `struct DataPacket`.
3. **Envío LoRa:** Manda los datos por la antena (binario).
4. **Gestión de Energía:**
* Si es **EMERGENCIA**: Espera solo 0.2 segundos y vuelve a enviar (Ráfaga).
* Si es **NORMAL**: Espera 1 segundo y vuelve a enviar (Ahorro de batería).



#### Pista 3: El Guardaespaldas (`loop` en Core 1)

Esta es la tarea principal que vigila el movimiento y pinta la pantalla.

1. **Lectura de Movimiento:**
* Pide la llave (Mutex).
* Lee el Acelerómetro (MPU9250).
* Calcula la **Magnitud** (fuerza total) y la **Inclinación**.


2. **Detección de Impacto:**
* Si la fuerza > 40G (umbral alto), marca `detectedFall = true` y guarda la hora (`millis`).


3. **Análisis Post-Caída (`filtroCaidas`):**
* Si hubo impacto hace 3 segundos, verifica: ¿El paciente sigue en el suelo (inclinación > 70°)? ¿Está quieto (fuerza baja)?
* Si SÍ: Cambia `g_estadoGlobal` a **"CAIDA CONFIRMADA"**. (Esto activa la alarma en la Pista 2).
* Si NO: Dice "RECUPERADO".


4. **Pantalla OLED:**
* Pide la llave (Mutex).
* Dibuja los BPM, SpO2 y el Estado actual en la pantallita.

---

### Resumen del Flujo de Datos

1. El **Sensor de Pulso** llena el buffer (Core 0).
2. El **Core 0** calcula el número "80 BPM" y lo pone en una variable global.
3. El **Core 1 (Loop)** detecta que la persona se cayó y escribe "CAIDA" en la variable de estado.
4. El **Core 1 (LoRa Task)** lee "80 BPM" y "CAIDA", hace un paquete y lo dispara por la antena.
5. La **OLED** muestra todo esto visualmente al usuario.

### El papel crucial del `i2cMutex`

Notarás que cada vez que alguien quiere usar `particleSensor`, `mpu`, `bme` o `display`, hace esto:
`if (xSemaphoreTake(i2cMutex...))`

Esto es vital. Si el **Médico** (Core 0) está leyendo el pulso y al mismo tiempo el **Guardaespaldas** (Core 1) intenta escribir en la pantalla OLED, los cables I2C se cruzarían y el programa colapsaría. El Mutex obliga a que hagan fila: *"Espera tu turno para usar el cable"*.