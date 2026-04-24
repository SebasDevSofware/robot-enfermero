La comunicacion entre placas no esta funcionando
bien aun

## Lo que debe hacer el codigo (flujo) :

- Obtener los ultimos 5 paquetes de datos enviados de la pulsera

- Hacer un promedio de esos ultimos 5 paquetes para evitar ruido y falsas alarmas, el resto del flujo trabajaria con este paquete ya analizado.

- Los datos pasan por una Fuzzy Logic IA para dar un diagnostico completo del estado del paciente y mostrarlo en un dasboard web a tiempo real

- El analisis de la IA tambien debe hacer inferencias de datos y un analisis de promedios

- Si el robot detecta valores de emergencia (ej. caidas o problemas cardiacos) activara el buzzer y empezara a dar vueltas como emergencia, ademas de alertar en el dashboard web y a traves de alertas de WhatsApp y activar uno de los tres led de colores (verde, amarillo o rojo) indicando el eestado del paciente (mientras que el paciente este estable seria verde encendido constantemente).