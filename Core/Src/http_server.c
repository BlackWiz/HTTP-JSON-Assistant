#include "http_server.h"
#include "webpage.h"
#include "main.h" // For LED_BLUE_Pin definitions
#include "lwip/debug.h"
#include "lwip/stats.h"
#include "lwip/tcp.h"
#include "lwip/err.h"   // <--- This fixes 'unknown type name err_t'
#include <string.h>
#include <stdio.h>

/* The Server Control Block */
static struct tcp_pcb *http_pcb;

/* Structure to track connection state (reused from echo example) */
struct http_state {
    uint8_t retries;
};

/* Forward declarations */
static err_t http_accept(void *arg, struct tcp_pcb *newpcb, err_t err);
static err_t http_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err);
static void http_conn_err(void *arg, err_t err);
static err_t http_poll(void *arg, struct tcp_pcb *tpcb);
static void http_close(struct tcp_pcb *tpcb, struct http_state *hs);

/**
 * @brief  Initializes the HTTP server on Port 80
 */
void http_server_init(void)
{
    http_pcb = tcp_new();
    if (http_pcb != NULL)
    {
        err_t err;

        // Bind to Port 80 (HTTP Standard)
        err = tcp_bind(http_pcb, IP_ADDR_ANY, 80);

        if (err == ERR_OK)
        {
            http_pcb = tcp_listen(http_pcb);
            tcp_accept(http_pcb, http_accept);
        }
        else
        {
            memp_free(MEMP_TCP_PCB, http_pcb);
        }
    }
}

static err_t http_accept(void *arg, struct tcp_pcb *newpcb, err_t err)
{
    struct http_state *hs;

    LWIP_UNUSED_ARG(arg);
    LWIP_UNUSED_ARG(err);

    // Set priority for the connection
    tcp_setprio(newpcb, TCP_PRIO_MIN);

    // Allocate structure to track this connection
    hs = (struct http_state *)mem_malloc(sizeof(struct http_state));
    if (hs != NULL)
    {
        hs->retries = 0;

        // Pass 'hs' as the callback argument
        tcp_arg(newpcb, hs);

        // Register the Callbacks
        tcp_recv(newpcb, http_recv);
        tcp_err(newpcb, http_conn_err);
        tcp_poll(newpcb, http_poll, 4); // Poll every 2s

        return ERR_OK;
    }
    else
    {
        return ERR_MEM;
    }
}

static err_t http_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
    struct http_state *hs = (struct http_state *)arg;
    char *data;

    if (p == NULL)
    {
        // Connection closed by client
        http_close(tpcb, hs);
        return ERR_OK;
    }
    else if (err != ERR_OK)
    {
        // Cleanup on error
        if (p != NULL) pbuf_free(p);
        return err;
    }

    // --- HTTP PARSER LOGIC ---

    // 1. Point to the payload
    data = (char *)p->payload;

    // 2. Check for GET Request (Load Page)
    if (strncmp(data, "GET / ", 6) == 0 || strncmp(data, "GET /index.html", 15) == 0)
    {
        // Send the HTML page from webpage.h
        tcp_write(tpcb, index_html, sizeof(index_html)-1, TCP_WRITE_FLAG_COPY);
    }
    // 3. Check for POST Request (Button Press)
    else if (strncmp(data, "POST /api/cmd", 13) == 0)
    {
        // Simple String Search for JSON (Not robust, but fast for MVP)
        if (strstr(data, "\"cmd\":\"ON\""))
        {
            // Turn LED ON (Using the pin name from your project)
            // Adjust LED_BLUE_GPIO_Port if your board uses LD2 or PA5
            HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET);
        }
        else if (strstr(data, "\"cmd\":\"OFF\""))
        {
            // Turn LED OFF
            HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);
        }

        // Send 200 OK
        char resp[] = "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\n\r\n{\"status\":\"ok\"}";
        tcp_write(tpcb, resp, strlen(resp), TCP_WRITE_FLAG_COPY);
    }
    else
    {
        // 404 Not Found
        char resp[] = "HTTP/1.1 404 Not Found\r\n\r\n";
        tcp_write(tpcb, resp, strlen(resp), TCP_WRITE_FLAG_COPY);
    }

    // 4. Send immediately
    tcp_output(tpcb);

    // 5. Advertise window size (We read the data)
    tcp_recved(tpcb, p->tot_len);

    // 6. Free the buffer
    pbuf_free(p);

    // 7. Close connection (Simple HTTP 1.0 style)
    http_close(tpcb, hs);

    return ERR_OK;
}

static void http_close(struct tcp_pcb *tpcb, struct http_state *hs)
{
    tcp_arg(tpcb, NULL);
    tcp_sent(tpcb, NULL);
    tcp_recv(tpcb, NULL);
    tcp_err(tpcb, NULL);
    tcp_poll(tpcb, NULL, 0);

    if (hs != NULL) mem_free(hs);

    tcp_close(tpcb);
}

static void http_conn_err(void *arg, err_t err)
{
    struct http_state *hs = (struct http_state *)arg;
    LWIP_UNUSED_ARG(err);
    if (hs != NULL) mem_free(hs);
}

static err_t http_poll(void *arg, struct tcp_pcb *tpcb)
{
    struct http_state *hs = (struct http_state *)arg;
    http_close(tpcb, hs); // Auto-close idle connections
    return ERR_OK;
}
