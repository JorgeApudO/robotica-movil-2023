#  Prio
1. Tiro Particulas (RANDOM)

# FLUJO GENERAL
2. Sensor
3. Particulas
4. Check Finalizacion (Jump 7)
5. Movimiento
6. Jump 2
7. Notificar 

Sensor:
- Toma datos del lidar
- Publica datos del lidar al modelo de sensor

Particulas:
- Se aplica el modelo de particulas y se resamplea la muestra

Check Finalizacion:
- Revisa si ya está localizado
- Si está localizado salta a Notificar
- Si no está localizado se mueve un poquito
    

Movimiento:
- Se mueve reactivamente 20 cm
- para si tiene muro delante y se endereza manteniendo distancia con el muro


Notificar:
- Avisa por los parlantes que está localizado y su ubicacion

--------------------------------------------------------------------------------





1. Inicio quieto

2. Escaneo

3. Reviso si puedo localizarme

    1. Si puedo localizarme me salto #4 y #5

4. Avanzo medi
5. Mientras no esté localizado

    1. Giro a la derecha
    2. Avanzo hasta chocar
    3. Escaneo

6. Notificar donde estoy

## Movimiento
- Notificar al filtro de partículas el movimiento