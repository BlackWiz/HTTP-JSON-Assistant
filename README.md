# HTTP-JSON-Assistant

## Overview

HTTP-JSON-Assistant is a bare-metal embedded system built on an STM32 microcontroller that receives commands over HTTP, parses JSON payloads, and executes deterministic hardware actions.

The project exists to explore an alternative to modern reminder and alarm systems that are tightly coupled to smartphones and cloud services. Such systems depend on continuous connectivity, background execution privileges, battery availability, and operating system notification policies. As a result, routine or time-critical reminders often fail for reasons unrelated to the actual schedule.

This project takes a different approach. A dedicated embedded system assumes responsibility for real-time execution, while external devices are used only to provide configuration data and schedules. Once configured, the embedded system operates independently and reliably.

The project is developed as a single evolving system. Its long-term goal is to function as a standalone alarm and reminder assistant driven by time-based events. Development is intentionally phased so that the communication and execution foundation is validated before higher-level assistant behavior is added.

---
## Current Focus (Phase-1)

Phase-1 is limited to validating the communication and execution pipeline.

* STM32 acts as a network-accessible endpoint
* Commands are received over HTTP
* JSON payloads are parsed deterministically
* A simple hardware action (LED blink) is executed

The LED is used only to verify the end-to-end path and is not a feature.
