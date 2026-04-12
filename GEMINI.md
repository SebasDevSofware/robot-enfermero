# ALMA Project Architecture & Coding Guidelines (GEMINI.md)

## 1. Filosofía de Desarrollo
* **Estabilidad Crítica:** ALMA es un dispositivo médico/telemetría. El código C++ en el ESP32 utiliza FreeRTOS. **NO modifiques** la lógica de los Mutex (`i2cMutex`, `dataMutex`), las prioridades de las tareas, ni el Filtro Madgwick a menos que se solicite explícitamente.
* **Aislamiento del Frontend:** La interfaz web está incrustada en PROGMEM (`index_html`). Cualquier adición interactiva o gamificación debe resolverse **100% en el lado del cliente (Vanilla JS, CSS, HTML)** para no cargar el Core 0 del ESP32 con procesamiento innecesario.
* **Arquitectura Limpia en JS:** Aunque no estemos usando un framework moderno para el dashboard embebido, el Vanilla JS debe emular principios de separación de responsabilidades: funciones puras para el cálculo matemático y manipulación del DOM aislada.

## 2. Estilo Visual (UI/UX)
* **Tema:** Oscuro, médico/cyberpunk.
* **Colores Principales:** Fondo `#0a0a0a`, Texto principal `#00ffcc`, Alertas `#ff3333`, Advertencias `#ff9900`.
* **Tipografía:** `monospace` (Courier New).