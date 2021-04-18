/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>

#include "protocol_examples_common.h"
#include "sdkconfig.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_event_loop.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

#include "tcpip_adapter.h"

#include "driver/gpio.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "esp_netif.h"
#include "nvs.h"
#include "nvs_flash.h"

#include <esp_http_server.h>
#include <http_parser.h>

#include "arducam_commun.h"

//#include "ov2640_regs.h"

#define TAG "main"

EventGroupHandle_t arducam_init_done_grpevt;


#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART_FORMAT = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";


static inline void cam_re_init_polling()
{
	CAM_CS_END;
	i2c_driver_delete(I2C_NUM_0);
	spi_deinit(HSPI_HOST);	
	xEventGroupSetBits(arducam_init_done_grpevt, BIT(0)); // set for arducam re init 
	xEventGroupClearBits(arducam_init_done_grpevt, BIT(1)); // clear for wait init 
    xEventGroupWaitBits(arducam_init_done_grpevt, BIT(1), true, false, portMAX_DELAY); // wait for init done
}

uint8_t cam_spi_check()
{
	uint8_t testdata[4] = {0xAA,0xBB,0xCC,0xDD};
	
	for(int i=0;i<4;i++)
	{
		spi_write_reg(0x00, testdata[i]);
		if(spi_read_reg(0x00) != testdata[i])
		{
			ESP_LOGE(TAG, "cam spi check failed");
			return 1;
		}
	}

	return 0;
}

esp_err_t hello_type_get_handler(httpd_req_t *req)
{
	size_t qrylen = httpd_req_get_url_query_len(req);
	char *bufurl = malloc(qrylen+1);
	httpd_req_get_url_query_str(req, bufurl, qrylen+1);
	bufurl[qrylen] = 0;

	ESP_LOGI(TAG, "url_query_str - %s", bufurl);

	if(strcmp(bufurl, "caminit") == 0)
	{
		cam_re_init_polling();
		const char *STR = "cam reset";
		httpd_resp_set_type(req, HTTPD_TYPE_TEXT);
		httpd_resp_send(req, STR, strlen(STR));
	}
	else if(strcmp(bufurl, "img") == 0)
	{
		spi_write_reg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
		spi_write_reg(ARDUCHIP_FIFO, FIFO_START_MASK);

		ESP_LOGI(TAG, "wait for capture");
		uint32_t timeout = 0;
		while( !(spi_read_reg(ARDUCHIP_TRIG) & CAP_DONE_MASK)) {
			vTaskDelay(50 / portTICK_RATE_MS);
			timeout++;
			if(timeout >= 40) break;
		}
		if(timeout == 40)
		{
			const char *STR = "cam error";
			httpd_resp_set_type(req, HTTPD_TYPE_TEXT);
			httpd_resp_send(req, STR, strlen(STR));
		}
		else
		{
			ESP_LOGI(TAG, "capture done");
			
			uint32_t bufurlfered_fifosize=0;
			
			bufurlfered_fifosize |= spi_read_reg(FIFO_SIZE3);
			bufurlfered_fifosize <<= 8;
			bufurlfered_fifosize |= spi_read_reg(FIFO_SIZE2);
			bufurlfered_fifosize <<= 8;
			bufurlfered_fifosize |= spi_read_reg(FIFO_SIZE1);

			if(bufurlfered_fifosize > MAX_FIFO_SIZE)
			{
				ESP_LOGI(TAG, "ERROR - captured size : %d bytes reached max MAX_FIFO_SIZE", bufurlfered_fifosize);
				
				const char *STR = "img error";
				httpd_resp_set_type(req, HTTPD_TYPE_TEXT);
				httpd_resp_send(req, STR, strlen(STR));
			}
			else
			{
				ESP_LOGI(TAG, "captured size : %d bytes", bufurlfered_fifosize);
				
				httpd_resp_set_type(req, HTTPD_TYPE_IMAGE_JPEG);
				httpd_resp_send_hdr_only(req, bufurlfered_fifosize); // this is deprecate by using httpd_resp_send_chunk

				const int bufurllen = 128;
				char *txbufurl = malloc(bufurllen);
				
				CAM_CS_BEGIN;
				spi_transfer(BURST_FIFO_READ);
				uint8_t bytenow=0,bytepast=0;
				uint32_t bufurlindex = 0, sendsize=0;
				while(bufurlfered_fifosize--)
				{
					bytenow = spi_transfer(0x00);
					txbufurl[bufurlindex] = bytenow;
					bufurlindex++;
					sendsize++;
					
					if(bufurlindex >= 128)
					{
						//if(httpd_resp_send_chunk(req, txbufurl, bufurlindex) != ESP_OK)
						if(httpd_send(req, txbufurl, bufurlindex) < 0)
						{
							ESP_LOGE(TAG, "Client Closed Connection");
							break;
						}
						bufurlindex=0;
					}
					
					if(bytenow == 0xD9 && bytepast == 0xFF)
					{
						ESP_LOGI(TAG, "FFD9 found at %d", sendsize);
					}
					
					bytepast = bytenow;

					//vTaskDelay(1 / portTICK_RATE_MS);
					
				}

				if(bufurlindex > 0)
				{
					//if(httpd_resp_send_chunk(req, txbufurl, bufurlindex) != ESP_OK)
					if(httpd_send(req, txbufurl, bufurlindex) < 0)
					{
						ESP_LOGE(TAG, "Client Closed Connection");
					}
				}
				CAM_CS_END;
				spi_write_reg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);

				ESP_LOGI(TAG, "%d bytes sent", sendsize);
				free(txbufurl);
			}
		}
	}
	else if(strcmp(bufurl, "stream") == 0)
	{
		httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
	
		uint8_t stream_terminated = 0;
		const int bufurllen = 128;
		char *txbufurl = malloc(bufurllen);
		char *_stream_part = malloc(64);
		while(stream_terminated == 0)
		{
			spi_write_reg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
			spi_write_reg(ARDUCHIP_FIFO, FIFO_START_MASK);
			
			ESP_LOGI(TAG, "wait for capture");
			uint32_t timeout = 0;
			while( !(spi_read_reg(ARDUCHIP_TRIG) & CAP_DONE_MASK)) {
				vTaskDelay(50 / portTICK_RATE_MS);
				timeout++;
				if(timeout >= 40) break;
			}

			
			if(timeout == 40)
			{
				ESP_LOGE(TAG, "wait for capture timeout");
				stream_terminated = 1;
			}
			else
			{
				ESP_LOGI(TAG, "capture done");
				
				uint32_t bufurlfered_fifosize=0;
				
				bufurlfered_fifosize |= spi_read_reg(FIFO_SIZE3);
				bufurlfered_fifosize <<= 8;
				bufurlfered_fifosize |= spi_read_reg(FIFO_SIZE2);
				bufurlfered_fifosize <<= 8;
				bufurlfered_fifosize |= spi_read_reg(FIFO_SIZE1);

				sprintf(_stream_part, _STREAM_PART_FORMAT, bufurlfered_fifosize);
				
				if(httpd_resp_send_chunk(req, _stream_part, strlen(_stream_part)) != ESP_OK)
				{
					ESP_LOGE(TAG, "Client Closed Connection");
					stream_terminated = 1;				}
				else 
				if(bufurlfered_fifosize < MAX_FIFO_SIZE)
				{
					ESP_LOGI(TAG, "captured size : %d bytes", bufurlfered_fifosize);
					
				
					CAM_CS_BEGIN;
					spi_transfer(BURST_FIFO_READ);
					uint8_t bytenow=0,bytepast=0;
					uint32_t bufurlindex = 0, sendsize=0;
					while(bufurlfered_fifosize--)
					{
						bytenow = spi_transfer(0x00);
						txbufurl[bufurlindex] = bytenow;
						bufurlindex++;
						sendsize++;
						
						if(bufurlindex >= 128)
						{
							if(httpd_resp_send_chunk(req, txbufurl, bufurlindex) != ESP_OK)
							{
								ESP_LOGE(TAG, "Client Closed Connection");
								stream_terminated = 1;
								break;
							}
							bufurlindex=0;
						}
						
						if(bytenow == 0xD9 && bytepast == 0xFF)
						{
							ESP_LOGI(TAG, "FFD9 found at %d", sendsize);
						}
						
						bytepast = bytenow;
					
						//vTaskDelay(1 / portTICK_RATE_MS);
						
					}
					
					if(bufurlindex > 0)
					{
						if(httpd_resp_send_chunk(req, txbufurl, bufurlindex) != ESP_OK)
						{
							ESP_LOGE(TAG, "Client Closed Connection");
							stream_terminated = 1;
						}
					}
					
					CAM_CS_END;
					spi_write_reg(ARDUCHIP_FIFO, FIFO_CLEAR_MASK);
					ESP_LOGI(TAG, "%d bytes sent", sendsize);

					if(httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY)) != ESP_OK)
					{
						ESP_LOGE(TAG, "Client Closed Connection");
						stream_terminated = 1;
					}
					
				}
				else
				{
					ESP_LOGI(TAG, "MAX_FIFO_SIZE reached");
					stream_terminated = 1;
				}
			}
		}
	
		free(txbufurl);

	}
	else if(strncmp(bufurl, "resolution", 10) == 0)
	{
		char STR[32];

		if(strlen(bufurl) == 11)
		{
			uint8_t arg = bufurl[10]-'0';
			ESP_LOGI(TAG, "arg:%d", arg);
			if(arg<7)
			{
				arducam_jpeg_change_resolution(arg);
				vTaskDelay(1000 / portTICK_RATE_MS);
				sprintf(STR, "resized:%d", arg);
			}
			else
			{
				sprintf(STR, "resize arg error");
			}
		}
		else
		{
			sprintf(STR, "resize arg error");
		}
		
		httpd_resp_set_type(req, HTTPD_TYPE_TEXT);
		httpd_resp_send(req, STR, strlen(STR));

	}
	else
	{
		const char *STR = "default";
		httpd_resp_set_type(req, HTTPD_TYPE_TEXT);
		httpd_resp_send(req, STR, strlen(STR));
	}

	ESP_LOGI(TAG, "client session terminated");
	
    free (bufurl);
	return ESP_OK;
}



httpd_uri_t basic_handlers = {
	.uri = "/test",
	.method = HTTP_GET,
	.handler = hello_type_get_handler,
	.user_ctx = NULL
};

void register_basic_handlers(httpd_handle_t hd)
{
    ESP_LOGI(TAG, "Registering basic handlers");
    if (httpd_register_uri_handler(hd, &basic_handlers) != ESP_OK) {

        ESP_LOGW(TAG, "register uri failed");
        return;
    }
    ESP_LOGI(TAG, "Success");
}


int pre_start_mem, post_stop_mem;
httpd_handle_t start_httpd()
{
    pre_start_mem = esp_get_free_heap_size();
    httpd_handle_t hd = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 8080;

    /* This check should be a part of http_server */
    config.max_open_sockets = (CONFIG_LWIP_MAX_SOCKETS - 3);

    if (httpd_start(&hd, &config) == ESP_OK) {
        ESP_LOGI(TAG, "Started HTTP server on port: '%d'", config.server_port);
        ESP_LOGI(TAG, "Max URI handlers: '%d'", config.max_uri_handlers);
        ESP_LOGI(TAG, "Max Open Sessions: '%d'", config.max_open_sockets);
        ESP_LOGI(TAG, "Max Header Length: '%d'", HTTPD_MAX_REQ_HDR_LEN);
        ESP_LOGI(TAG, "Max URI Length: '%d'", HTTPD_MAX_URI_LEN);
        ESP_LOGI(TAG, "Max Stack Size: '%d'", config.stack_size);
    }
	
    if (hd) {
        register_basic_handlers(hd);
    }
	
    return hd;
}

void stop_httpd(httpd_handle_t hd)
{
    ESP_LOGI(TAG, "Stopping httpd");
    httpd_stop(hd);
    post_stop_mem = esp_get_free_heap_size();
    ESP_LOGI(TAG, "HTTPD Stop: Current free memory: %d", post_stop_mem);
}


void wifi_event_got_ip(httpd_handle_t* server)
{
    if (*server == NULL) {
        ESP_LOGI(TAG, "Starting webserver");
        *server = start_httpd();
    }
}

void wifi_event_disconnected(httpd_handle_t* server)
{
    if (*server) {
        ESP_LOGI(TAG, "Stopping webserver");
        stop_httpd(*server);
        *server = NULL;
    }
}



static inline void arducam_camera_init(void)
{
	gpio_set_direction(ARDUCAM_CS_PIN, GPIO_MODE_OUTPUT);
	CAM_CS_END;
	
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = 4;
    conf.sda_pullup_en = 1;
    conf.scl_io_num = 5;
    conf.scl_pullup_en = 1;
    conf.clk_stretch_tick = 300;

	i2c_driver_install(I2C_NUM_0, conf.mode);
	i2c_param_config(I2C_NUM_0, &conf);
	
	spi_config_t spi_config;
    spi_config.interface.val = SPI_DEFAULT_INTERFACE;
	spi_config.interface.cs_en=0;
	spi_config.mode = SPI_MASTER_MODE;
	spi_config.clk_div = SPI_8MHz_DIV;
	spi_init(HSPI_HOST, &spi_config);
	
    SPI1C = 0;
    SPI1U = SPIUMOSI | SPIUDUPLEX | SPIUSSE;
    SPI1U1 = (7 << SPILMOSI) | (7 << SPILMISO);
    SPI1C1 = 0;
	setBitOrder(1);

	uint8_t res;
	while((res=ardu_cam_init())!=ESP_OK)
	{
		if(res == 1)
			ESP_LOGE(TAG, "spi test communication error...");
		else if(res == 2)
			ESP_LOGE(TAG, "cam sensor id not match...");
			
		vTaskDelay(1000 / portTICK_RATE_MS);
	}
}

void app_main()
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
	example_set_connection_info("ipTIMEA2003NS", "hyeon__1123");
	ESP_ERROR_CHECK(example_connect());

	arducam_init_done_grpevt = xEventGroupCreate();
	xEventGroupSetBits(arducam_init_done_grpevt, BIT(0));
	
	while(1)
	{
	    xEventGroupWaitBits(arducam_init_done_grpevt, BIT(0), true, false, portMAX_DELAY);
		arducam_camera_init();
		ESP_LOGI(TAG, "ARDUCAM INIT SUCCESS");
		xEventGroupSetBits(arducam_init_done_grpevt, BIT(1));
	}
	
}
