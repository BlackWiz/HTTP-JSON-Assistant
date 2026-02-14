# HTTP-Ethernet Assistant

[![Project Website](https://img.shields.io/badge/View_Project_Website-black?style=for-the-badge&logo=github&logoColor=white)](https://blackwiz.github.io/HTTP-JSON-Assistant/)

**Network-controlled hardware assistant that parses JSON commands to execute real-time tasks.**

**Unique Approach:** Implemented twice—first using industry-standard HAL for rapid prototyping, then rewritten in pure bare-metal C to master low-level driver architecture.

---

## 🆚 Dual-Implementation Strategy

I maintain two active branches to demonstrate the difference between "Application Development" and "System Engineering."

### [The Bare-Metal Engine (Dev Branch)](https://github.com/BlackWiz/HTTP-JSON-Assistant/tree/dev)
**"The Deep Dive"**

* **Philosophy:** Zero abstraction. Direct register access.
* **Why:** To understand exactly how ENC28J60 SPI protocol and LwIP stack interact without HAL overhead.
* **Key Tech:** Manual SPI driver, custom interrupt handlers, register-level GPIO, LwIP (NO_SYS).
* **Status:** Active development
* [**👉 View Bare-Metal Code**](https://github.com/BlackWiz/HTTP-JSON-Assistant/tree/dev)

### [The HAL Reference (Main Branch)](https://github.com/BlackWiz/HTTP-JSON-Assistant/tree/main)
**"The Prototype"**

* **Philosophy:** Rapid development using STM32CubeHAL.
* **Why:** Validated hardware connections, network logic, and ThingSpeak API integration before optimizing.
* **Key Tech:** STM32 HAL, CubeMX configuration.
* **Status:** Stable reference implementation
* [**👉 View Reference Code**](https://github.com/BlackWiz/HTTP-JSON-Assistant/tree/main)

---

## 🏗️ System Architecture

* **Hardware Layer:** ENC28J60 Ethernet controller (SPI interface) connected to STM32G071RB
* **Network Layer:** LwIP TCP/IP stack configured in NO_SYS mode (event loop callback style)
* **Application Layer:** Custom HTTP 1.1 server + JSON command parser
* **Cloud Layer:** Integration with ThingSpeak for sensor data logging

---

## 🛠️ Tools & Debugging

Development relied heavily on network analysis tools to validate the custom stack:

* **Wireshark:** Capture raw Ethernet frames, verify TCP 3-way handshake, debug ARP/DHCP sequences
* **Packet Sender:** Critical for "Echo Testing"—send raw TCP packets to verify stack receives, processes, and returns data without crashing before attempting complex HTTP parsing
* **MobaXterm (Serial Monitor):** Primary debug console for real-time LwIP assertions, IP address assignment logs, and system status via UART (115200 baud)

---

## 🔌 Hardware Connections

*Pinout matches NUCLEO-G071RB configuration*

| ENC28J60 Signal | STM32 Pin | Arduino Header | Function |
|-----------------|-----------|----------------|----------|
| **SCK** | **PA1** | **A1** | SPI Clock |
| **MISO** | **PA6** | **D12** | SPI Master In Slave Out |
| **MOSI** | **PA7** | **D11** | SPI Master Out Slave In |
| **CS** | **PA5** | **D13** | Chip Select |
| **INT** | **PB0** | **D3** | Interrupt (Optional) |
| **RESET** | **PB1** | **-** | Hardware Reset |
| **VCC** | **5V** | **5V** | Power |
| **GND** | **GND** | **GND** | Ground |

---

## 🚀 Roadmap

This project is evolving from a simple network endpoint to a fully functional IoT assistant.

### Phase 1: Basic Infrastructure (Current Focus)
* **ThingSpeak Integration:** Implementing REST API client to upload sensor data to cloud—establishes basic "uplink" capability
* **Stability:** Stress-testing LwIP stack to handle continuous data transmission without memory leaks

### Phase 2: Smart Assistant Logic (Next Steps)
* **Google Calendar API:** Device connects to Google Calendar REST API to fetch daily schedules
* **Advanced JSON Parsing:** Parse complex nested JSON responses from Google to extract event times and summaries
* **Hardware Alarms:** Trigger physical alerts (buzzer/LED) based on fetched calendar events

---

## 🧪 Testing the System

### 1. Monitor Serial Debug (MobaXterm)
1. Open **MobaXterm**
2. Click **Session** → **Serial**
3. Select the STM32 COM Port
4. Set baud rate to **115200**
5. **Result:** You should see `IP Assigned: 192.168.0.200` (or similar) in logs

### 2. Verify Network Connectivity
```bash
# Ping the device from your PC terminal
ping 192.168.0.200
```

### 3. TCP Echo Test (Packet Sender)

Before trying HTTP, validate the stack using raw TCP Echo test.

* **Tool:** Packet Sender
* **Address:** `192.168.0.200`
* **Port:** `7` (Echo Protocol)
* **Message:** `Hello STM32`
* **Click:** Send
* **Expected Result:** Device receives packet and immediately sends back `Hello STM32`

### 4. Control via Web Browser

Once Echo test passes, use the HTTP server.

1. Open web browser (Chrome/Edge/Firefox)
2. Type the IP address: `http://192.168.0.200`
3. **Result:** You will see custom control page hosted by STM32
4. **Action:** Click "Toggle LED" button to control hardware in real-time

---

## ⚔️ The Hard Parts

**LwIP in NO_SYS Mode:** Integrating full TCP/IP stack without RTOS meant manually managing `sys_check_timeouts()` loop. Had to ensure Ethernet interrupts didn't corrupt main loop's packet processing.

**The SPI Bottleneck:** ENC28J60 is an SPI device, not memory-mapped. Optimized SPI driver to handle 10Mbps traffic without stalling CPU.

**Parsing JSON in C:** Standard libraries too heavy. Implemented token-based parser (JSMN style) to read `{ "cmd": "led", "val": 1 }` directly from TCP buffer without malloc.

---

*Built by Sri Hari Shankar Sharma. Part of the transition from Automotive Applications to Embedded Systems Engineering.*
