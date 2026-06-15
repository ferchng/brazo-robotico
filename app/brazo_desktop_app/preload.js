const { contextBridge, ipcRenderer } = require("electron");

contextBridge.exposeInMainWorld("desktopAPI", {
  listPorts: () => ipcRenderer.invoke("serial:list"),
  connectSerial: payload => ipcRenderer.invoke("serial:connect", payload),
  disconnectSerial: () => ipcRenderer.invoke("serial:disconnect"),
  writeSerial: text => ipcRenderer.invoke("serial:write", text),
  showError: payload => ipcRenderer.invoke("dialog:error", payload),
  onSerialLine: callback => ipcRenderer.on("serial:line", (_event, line) => callback(line)),
  onSerialError: callback => ipcRenderer.on("serial:error", (_event, message) => callback(message)),
  onSerialClosed: callback => ipcRenderer.on("serial:closed", () => callback())
});
