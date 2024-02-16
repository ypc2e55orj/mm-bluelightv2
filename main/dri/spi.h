#pragma once

// C++
#include <vector>

// ESP-IDF
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_intr_alloc.h>

class Spi {
 private:
  struct SpiDevice {
    spi_device_handle_t handle = nullptr;
    gpio_num_t spics_io_num = GPIO_NUM_NC;
    spi_transaction_t *transaction = nullptr;
  };

  spi_host_device_t host_id_;
  std::vector<SpiDevice *> devices_;

 public:
  explicit Spi(spi_host_device_t host_id, gpio_num_t miso_io_num, gpio_num_t mosi_io_num, gpio_num_t sclk_io_num,
               int max_transfer_sz)
      : host_id_(host_id), devices_() {
    spi_bus_config_t bus_config = {};
    bus_config.mosi_io_num = mosi_io_num;
    bus_config.miso_io_num = miso_io_num;
    bus_config.sclk_io_num = sclk_io_num;
    bus_config.max_transfer_sz = max_transfer_sz;
    bus_config.flags = SPICOMMON_BUSFLAG_MASTER;
    bus_config.intr_flags = ESP_INTR_FLAG_IRAM;

    ESP_ERROR_CHECK(spi_bus_initialize(host_id, &bus_config, SPI_DMA_CH_AUTO));
  }
  ~Spi() {
    ESP_ERROR_CHECK(spi_bus_free(host_id_));
    for (const auto device : devices_) {
      free(device->transaction);
      delete device;
    }
  }

  int add(uint8_t command_bits, uint8_t address_bits, uint8_t mode, int clock_speed_hz, gpio_num_t spics_io_num,
          int queue_size) {
    auto device = new SpiDevice();
    device->spics_io_num = spics_io_num;
    device->transaction = (spi_transaction_t *)heap_caps_calloc(1, sizeof(spi_transaction_t), MALLOC_CAP_DMA);
    device->transaction->user = device;

    spi_device_interface_config_t device_interface_config = {};
    device_interface_config.command_bits = command_bits;
    device_interface_config.address_bits = address_bits;
    device_interface_config.mode = mode;
    device_interface_config.clock_speed_hz = clock_speed_hz;
    device_interface_config.spics_io_num = spics_io_num;
    device_interface_config.queue_size = queue_size;

    if (spi_bus_add_device(host_id_, &device_interface_config, &device->handle) != ESP_OK) {
      free(device->transaction);
      delete device;
      return -1;
    }

    devices_.push_back(device);
    return static_cast<int>(devices_.size() - 1);
  }
  bool transmit(int index) {
    auto device = devices_[index];
    esp_err_t transmit_err = spi_device_transmit(device->handle, device->transaction);
    return transmit_err == ESP_OK;
  }
  spi_transaction_t *transaction(int index) { return devices_[index]->transaction; }
};
