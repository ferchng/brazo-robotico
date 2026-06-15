# Visualizador 2D simplificado del brazo

Objetivo:
- Ver visualmente la geometria simplificada antes de meternos con colisiones.
- Ajustar longitudes, offsets y sentido de ejes.

Modelo actual:
- Base fija:
  - `Hbase = 100 mm`
  - `Wbase = 35 mm`
- Tramo 1:
  - `L1 = 113 mm`
  - `W1 = 25 mm`
- Offset en M3:
  - `offset_M3 = 27 mm`
- Tramo 2:
  - `L2 = 137 mm`
  - `W2 = 25 mm`
- Tramo 3:
  - `L3 = 60 mm`
  - `W3 = 25 mm`
- Bloque simplificado de garra:
  - `largo = 100 mm`
  - `ancho = 65 mm`
  - anclado en `M4` desde el vertice libre mas alejado
  - con umbral de `PISO = 5 mm`
  - con umbral de `RIESGO PISO = 30 mm`

Interpretacion actual:
- El eje `M2` esta a altura `Hbase`.
- `L1` sale desde `M2`.
- En el extremo de `L1`, se aplica un offset perpendicular simplificado de `27 mm`.
- Desde ahi nace `L2`.
- `L3` representa la parte final hacia la pinza.
- Ademas se dibuja un bloque rigido de garra anclado en `M4`.
- Se dibuja tambien:
  - una linea de `PISO` a `5 mm`
  - una linea de `RIESGO PISO` a `30 mm`

Archivo:
- `visualizador_brazo.py`

Uso:
1. Tener Python instalado.
2. Instalar matplotlib si hace falta:
   - `py -m pip install matplotlib`
3. Ejecutar:
   - `py visualizador_brazo.py`

Si el dibujo no coincide con el brazo real:
- corregimos signos
- corregimos offset
- corregimos longitudes
