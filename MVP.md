# Phase-1 MVP — HTTP-JSON-Assistant

## Phase-1 Goal

Establish a stable and deterministic command execution pipeline on STM32.

External command → network → firmware → hardware action

Phase-1 focuses strictly on infrastructure correctness. No assistant or scheduling features are included.

---

## MVP-1: Hardware & Low-Level Drivers Bring-Up

### Scope
- STM32 clock and reset configuration
- GPIO driver (LED control)
- SPI driver
- ENC28J60 basic initialization and SPI communication

### Deliverables
- Firmware boots reliably
- LED can be toggled locally
- ENC28J60 responds over SPI

### Exit Criteria
- LED toggles via firmware
- SPI communication verified
- No crashes or undefined behavior

---

## MVP-2: Networking Stack Integration

### Scope
- lwIP integration (`NO_SYS = 1`)
- Network interface binding to ENC28J60
- Static IP configuration
- lwIP timer servicing

### Deliverables
- STM32 appears as a valid network node

### Exit Criteria
- STM32 responds to `ping`
- ARP resolution works
- No lockups under network traffic

---

## MVP-3: Transport & Application Protocol Handling

### Scope
- TCP server listening on port 80
- Connection accept, receive, and close handling
- Minimal HTTP request parsing
- Request body extraction

### Deliverables
- STM32 reliably receives HTTP requests

### Exit Criteria
- Client can connect using Postman / curl
- HTTP POST requests reach application layer
- Malformed HTTP does not crash firmware

---

## MVP-4: Command Parsing & Execution

### Scope
- Deterministic JSON parsing (FSM-based)
- Single supported command:
  {"cmd":"blink"}
- LED blink execution
- Fire-and-forget execution model
- Minimal HTTP acknowledgment (204 No Content)

### Deliverables
- End-to-end command execution

### Exit Criteria
- Valid command blinks LED
- Invalid JSON handled safely
- Connection closes cleanly

---

## MVP-5: System Testing & Stability Validation

### Scope
Validation of the complete Phase-1 system.

### Testing
- Functional testing (valid/invalid commands)
- Network testing (TCP handshake/teardown)
- Stability testing (power cycles, idle run, burst traffic)

### Tools
- Postman
- curl
- UART logs
- Wireshark (optional)

### Exit Criteria
- No crashes or lockups
- Deterministic behavior across resets
- System recovers cleanly from invalid inputs

---

## Out of Scope (Phase-1)

- RTC
- Scheduling logic
- Persistent storage
- Bluetooth or mobile integration
- Calendar synchronization
- User interfaces
