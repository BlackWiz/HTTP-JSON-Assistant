#ifndef INC_WEBPAGE_H_
#define INC_WEBPAGE_H_

/* The HTML content served when you visit the IP address */
const char index_html[] =
"HTTP/1.1 200 OK\r\n"
"Content-Type: text/html\r\n"
"Connection: close\r\n\r\n"
"<!DOCTYPE html>"
"<html>"
"<head><title>STM32 Control Panel</title>"
"<style>"
"body { font-family: sans-serif; text-align: center; margin-top: 50px; background-color: #f0f0f0; }"
".btn { padding: 20px 40px; font-size: 30px; margin: 20px; cursor: pointer; border-radius: 10px; border: none; }"
".on { background-color: #4CAF50; color: white; box-shadow: 0 5px #2E7D32; }"
".on:active { transform: translateY(4px); box-shadow: 0 1px #2E7D32; }"
".off { background-color: #f44336; color: white; box-shadow: 0 5px #c62828; }"
".off:active { transform: translateY(4px); box-shadow: 0 1px #c62828; }"
"</style></head>"
"<body>"
"<h1>STM32 HTTP Server</h1>"
"<h3>LED Control</h3>"
"<button class='btn on' onclick=\"sendCommand('ON')\">TURN ON</button>"
"<button class='btn off' onclick=\"sendCommand('OFF')\">TURN OFF</button>"
"<script>"
"function sendCommand(cmd) {"
"  fetch('/api/cmd', {"
"    method: 'POST',"
"    headers: {'Content-Type': 'application/json'},"
"    body: JSON.stringify({cmd: cmd})"
"  }).then(r => console.log('Sent:', cmd));"
"}"
"</script>"
"</body></html>";

#endif /* INC_WEBPAGE_H_ */
