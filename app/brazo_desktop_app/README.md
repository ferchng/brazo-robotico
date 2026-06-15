# Brazo Desktop App

App de escritorio en Electron para:

- visualizar el brazo en una simulación 3D local
- conectarse al ESP32 por puerto COM nativo
- leer `posall` y sincronizar la pose real
- mover motores desde sliders con cooldown
- aplicar validaciones de M2/M3/M4 antes de mover

## Requisitos

- Node.js 20+ recomendado
- Windows

## Instalar

```powershell
cd C:\proyecto\motores\brazo_desktop_app
npm install
```

## Ejecutar

```powershell
npm start
```

## Notas

- La app usa `serialport` nativo, no `Web Serial`.
- La simulación 3D es local y no depende de CDN ni navegador.
- El render y la lógica de reglas salen del prototipo web local ya validado.
