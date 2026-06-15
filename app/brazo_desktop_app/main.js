const { app, BrowserWindow, ipcMain, dialog } = require("electron");
const path = require("path");
const { SerialPort, ReadlineParser } = require("serialport");

let mainWindow = null;
let port = null;
let parser = null;

function sendToRenderer(channel, payload) {
  if (mainWindow && !mainWindow.isDestroyed()) {
    mainWindow.webContents.send(channel, payload);
  }
}

function createWindow() {
  mainWindow = new BrowserWindow({
    width: 1640,
    height: 1040,
    minWidth: 1320,
    minHeight: 840,
    backgroundColor: "#08101c",
    autoHideMenuBar: true,
    webPreferences: {
      preload: path.join(__dirname, "preload.js"),
      contextIsolation: true,
      nodeIntegration: false,
      sandbox: false
    }
  });

  mainWindow.loadFile(path.join(__dirname, "src", "index.html"));
}

async function closePort() {
  if (!port) return;
  const activePort = port;
  port = null;
  parser = null;
  await new Promise(resolve => {
    activePort.close(() => resolve());
  });
}

app.whenReady().then(() => {
  createWindow();

  app.on("activate", () => {
    if (BrowserWindow.getAllWindows().length === 0) createWindow();
  });
});

app.on("window-all-closed", async () => {
  await closePort();
  if (process.platform !== "darwin") app.quit();
});

ipcMain.handle("serial:list", async () => {
  const ports = await SerialPort.list();
  return ports.map(p => ({
    path: p.path,
    friendlyName: p.friendlyName || p.path,
    manufacturer: p.manufacturer || "",
    serialNumber: p.serialNumber || ""
  }));
});

ipcMain.handle("serial:connect", async (_event, { path: portPath, baudRate }) => {
  try {
    await closePort();

    port = new SerialPort({
      path: portPath,
      baudRate: Number(baudRate) || 115200,
      autoOpen: false
    });

    await new Promise((resolve, reject) => {
      port.open(err => err ? reject(err) : resolve());
    });

    parser = port.pipe(new ReadlineParser({ delimiter: "\r\n" }));
    parser.on("data", line => sendToRenderer("serial:line", line));
    port.on("error", err => sendToRenderer("serial:error", err.message));
    port.on("close", () => sendToRenderer("serial:closed", ""));

    return { ok: true };
  } catch (error) {
    await closePort();
    return { ok: false, error: error.message };
  }
});

ipcMain.handle("serial:disconnect", async () => {
  await closePort();
  return { ok: true };
});

ipcMain.handle("serial:write", async (_event, text) => {
  if (!port) return { ok: false, error: "No hay puerto abierto." };
  try {
    await new Promise((resolve, reject) => {
      port.write(String(text), err => err ? reject(err) : resolve());
    });
    return { ok: true };
  } catch (error) {
    return { ok: false, error: error.message };
  }
});

ipcMain.handle("dialog:error", async (_event, { title, message }) => {
  await dialog.showMessageBox(mainWindow, {
    type: "error",
    title: title || "Error",
    message: message || "Error desconocido"
  });
  return { ok: true };
});
