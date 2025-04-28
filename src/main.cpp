#include <Arduino.h>
#include <Audio.h>
//#include "BluetoothA2DPSink.h"
#include <driver/i2s.h>
#include <opus.h>
#include <RadioLib.h>
#include <SPI.h>
#include <stdint.h>

#define SAMPLE_RATE 48000

// Connections to INMP441 I2S microphone
#define I2S_WS 4
#define I2S_SD 35
#define I2S_SCK 19

// AMP
#define I2S_DIN 26
#define I2S_BCLK 19
#define I2S_LRC 21
#define I2S_CHANNEL_PIN 5 //SD

#define SPEAK_BUTTON 34

// Use I2S Processor 0
#define I2S_PORT_IN I2S_NUM_0
#define I2S_PORT_OUT I2S_NUM_1

#define AUDIO_SAMPLE_RATE     48000//8000  // 44100 init 8000
#define AUDIO_OPUS_FRAME_MS   20   // one of 2.5, 5, 10, 20, 40, 60, 80, 100, 120 init 40
#define AUDIO_OPUS_BITRATE    16000// bit rate from 2400 to 512000
#define AUDIO_OPUS_COMPLEXITY 0     // from 0 to 10
#define OPUS_FRAME_SIZE ((AUDIO_SAMPLE_RATE / 1000) * AUDIO_OPUS_FRAME_MS)

#define IDENTIFIER 2120

#define TASK_STACK_SIZE 16384

#define OUT_TO_IN_BUFF_FACTOR 5 //How much bigger the i2s_write buffer is relative to i2s_in

#define BUFFER_SIZE 960

#define LoRa_MOSI 10
#define LoRa_MISO 11
#define LoRa_SCK 9
#define LoRa_NSS 8
#define LoRa_DIO1 14
#define LoRa_NRST 12
#define LoRa_BUSY 13

#define LoRa_BANDWIDTH 500.0F
#define LoRa_CODING_RATE_DENOM 5
#define LoRa_FREQUENCY 920.0F
#define LoRa_POWER 17
#define LoRa_PREAMBLE_LEN 10U
#define LoRa_SPREADFACTOR 5
#define LoRa_SYNCWORD 0xF5
#define LoRa_TCXO_VOLTAGE 1.6
#define LoRa_USE_REGULATOR_LDO false

#define GAIN_FACTOR 2

//SX1262 radio = new Module(LoRa_NSS, LoRa_DIO1, LoRa_NRST, LoRa_BUSY);

Module *lora_module; //= new Module(LoRa_NSS, LoRa_DIO1, LoRa_NRST, LoRa_BUSY);

// Then pass it to SX1262
SX1262 *radio; //(lora_module);

OpusEncoder *opus_encoder_;
OpusDecoder *opus_decoder_;

void onSpeak();
void onReceive();
void normalize(int16_t*, size_t);
void setFlag();
//TaskFunction_t speak;

void i2s_install() {
  // Set up I2S Processor configuration
  const i2s_config_t i2s_config_in = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = i2s_bits_per_sample_t(16),//16
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
    .intr_alloc_flags = 0,
    .dma_buf_count = 16,
    .dma_buf_len = BUFFER_SIZE,
    .use_apll = false
  };

  const i2s_config_t i2s_config_out = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_TX),  // I2S Master mode and Transmit (TX)
    .sample_rate = SAMPLE_RATE,             // Set sample rate (16kHz)
    .bits_per_sample = i2s_bits_per_sample_t(16),     // Set audio bit depth (16-bit)
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT, // Stereo output (Left and Right channels)
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),  // I2S communication format
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,   // Interrupt level (default)
    .dma_buf_count = 16, //10
    .dma_buf_len = BUFFER_SIZE,
    .use_apll = false
  };

  esp_err_t err1 = i2s_driver_install(I2S_PORT_IN, &i2s_config_in, 0, NULL);
  esp_err_t err2 = i2s_driver_install(I2S_PORT_OUT, &i2s_config_out, 0, NULL);

if (err1 != ESP_OK) Serial.println("I2S Input install failed");
if (err2 != ESP_OK) Serial.println("I2S Output install failed");
}
 
void i2s_setpin() {
  // Set I2S pin configuration for mic
  const i2s_pin_config_t in_pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD,
    
  };

  const i2s_pin_config_t out_pin_config = {
    .bck_io_num = I2S_BCLK,
    .ws_io_num = I2S_LRC,
    .data_out_num = I2S_DIN,
    .data_in_num = I2S_CHANNEL_PIN
    
  };

  i2s_set_pin(I2S_PORT_IN, &in_pin_config);

  i2s_set_pin(I2S_PORT_OUT, &out_pin_config);
  i2s_zero_dma_buffer(I2S_PORT_OUT);
  
}

int16_t *new_opus;
uint8_t *compressed;
uint8_t *dataIn;
int16_t *uncompressed;


SemaphoreHandle_t speakSemaphore;
SemaphoreHandle_t receiveSemaphore;

volatile bool receivedFlag = false;
uint16_t last_received = millis();
//BluetoothA2DPSink a2dp_sink;
 
void setup() {
 
  /* Set up Serial Monitor */
  Serial.begin(115200);
  //while(!Serial);
  delay(3000);

  Serial.println("Serial ready");
  /* Set the speak button */
  pinMode(SPEAK_BUTTON, INPUT_PULLUP);


  i2s_install();
  Serial.println("I2S Install Complete");
  i2s_setpin();
  Serial.println("I2S Pins Set");

  pinMode(I2S_CHANNEL_PIN, OUTPUT);

  /* Set the SD pin on the amp to High */
  digitalWrite(I2S_CHANNEL_PIN, HIGH);

  i2s_start(I2S_PORT_IN);
  i2s_start(I2S_PORT_OUT);
  Serial.println("I2S Start");

  lora_module = new Module(LoRa_NSS, LoRa_DIO1, LoRa_NRST, LoRa_BUSY);
  radio = new SX1262(lora_module);

  int status = radio->begin(LoRa_FREQUENCY, 
              LoRa_BANDWIDTH, 
              LoRa_SPREADFACTOR, 
              LoRa_CODING_RATE_DENOM, 
              LoRa_SYNCWORD, 
              LoRa_POWER, 
              LoRa_PREAMBLE_LEN, 
              LoRa_TCXO_VOLTAGE, 
              LoRa_USE_REGULATOR_LDO);


              delay(100);

              Serial.println("begin");
              Serial.println(status);

              //radio.setDio1Action(setFlag);
              status = radio->startReceive();
              radio->setPacketReceivedAction(setFlag);
              Serial.println("Status");
              Serial.println(status);
              

  //delay(100);

  /* Create Encoder */
  int encoder_error;
  opus_encoder_ = opus_encoder_create(AUDIO_SAMPLE_RATE, 1, OPUS_APPLICATION_VOIP, &encoder_error);
  if (encoder_error != OPUS_OK) {
    Serial.println("ENCODE ERROR");
    Serial.println(encoder_error);
    return;
  }
  encoder_error = opus_encoder_init(opus_encoder_, AUDIO_SAMPLE_RATE, 1, OPUS_APPLICATION_VOIP);
  if (encoder_error != OPUS_OK) {
    Serial.println("ENCODE INIT ERROR");
    return;
  }
  Serial.println("Encoder Initialized");
  opus_encoder_ctl(opus_encoder_, OPUS_SET_BITRATE(AUDIO_OPUS_BITRATE));
  opus_encoder_ctl(opus_encoder_, OPUS_SET_COMPLEXITY(AUDIO_OPUS_COMPLEXITY));
  opus_encoder_ctl(opus_encoder_, OPUS_SET_SIGNAL(OPUS_SIGNAL_VOICE));
  opus_encoder_ctl(opus_encoder_, OPUS_SET_GAIN(0)); 
  opus_encoder_ctl(opus_encoder_, OPUS_SET_VBR(0));

  /* Create Decoder */
  int decoder_error;
  opus_decoder_ = opus_decoder_create(AUDIO_SAMPLE_RATE, 1, &decoder_error);
  if (decoder_error != OPUS_OK) {
    Serial.println("DECODE ERROR");
    Serial.println(decoder_error);
    return;
  } 

  new_opus = (int16_t*) malloc(OPUS_FRAME_SIZE * sizeof(int16_t));
  assert(new_opus != NULL);

  compressed = (uint8_t*) malloc(1275 * sizeof(uint8_t));
  assert(compressed != NULL);

  dataIn = (uint8_t*) malloc(1275 * sizeof(uint8_t));
  assert(dataIn != NULL);

  uncompressed = (int16_t*) malloc(OPUS_FRAME_SIZE * sizeof(int16_t));
  assert(uncompressed != NULL);
/*
  uncompressed2 = (int16_t*) malloc(OPUS_FRAME_SIZE * sizeof(int16_t));
  assert(uncompressed2 != NULL);

  uncompressed3 = (int16_t*) malloc(OPUS_FRAME_SIZE * sizeof(int16_t));
  assert(uncompressed3 != NULL);
  */

  speakSemaphore = xSemaphoreCreateBinary();
  xSemaphoreGive(speakSemaphore);

  receiveSemaphore = xSemaphoreCreateBinary();
  xSemaphoreGive(receiveSemaphore);

  Serial.print("SETUP COMPLETE\n");
  delay(100);
}
 
void loop() {

  if (digitalRead(SPEAK_BUTTON) == LOW) {

    if (xSemaphoreTake(speakSemaphore, 5) == pdTRUE) {
      //Serial.println("1");

      xTaskCreatePinnedToCore(
        [](void *){
          radio->standby();
          while (digitalRead(SPEAK_BUTTON) == LOW) {
            onSpeak();
            //yield();
          }
          radio->startReceive();
          xSemaphoreGive(speakSemaphore);
          vTaskDelete(NULL);
        },
        "speakTask",     // name
        8192 * 8,            // stack size in bytes
        NULL,            // param
        1,               // priority
        NULL,            // handle
        APP_CPU_NUM      // core
      );
    } 
  }

  //Will not work, must be interrupt driven instead (see last chat)
  //check if packet is available (doesn't actually read yet)
  if (receivedFlag && digitalRead(SPEAK_BUTTON) != LOW) {
    //Serial.println("Success");
    if (xSemaphoreTake(receiveSemaphore, 0) == pdTRUE) {


      xTaskCreatePinnedToCore(
        [](void *) {
          int count = 0;
          while (receivedFlag) {
            onReceive();
            receivedFlag = false;
            radio->startReceive();
            if (count > 2) {
              yield();
              count = 0;
            }
            else {
              count++;
            }

          }
          xSemaphoreGive(receiveSemaphore);
          vTaskDelete(NULL);
        },
        "receiveTask",     // name
        8192 * 8,            // stack size in bytes
        NULL,            // param
        1,               // priority
        NULL,            // handle
        0      // core
      );
    }
  }
  
}

void onReceive() {
  //Serial.println("Receiving");

  int status = radio->readData(dataIn, radio->getPacketLength());
  if (status == RADIOLIB_ERR_NONE) {
    //last_received = millis();
    /*
    int16_t *uncompressed;
    if (curr_buffer == 1) {
      uncompressed = uncompressed1;
      curr_buffer = 2;
      buffers_full++;
     // buf1 = true;
    }
    else if (curr_buffer == 2) {
      uncompressed = uncompressed2;
      curr_buffer = 3;
      buffers_full++;
      //buf2 = true;
    }
    else {
      uncompressed = uncompressed3;
      curr_buffer = 1;
      buffers_full++;
      //buf3 = true;
    }
      */

    int decoded_size = opus_decode(opus_decoder_, dataIn, radio->getPacketLength(), uncompressed, OPUS_FRAME_SIZE, 0);

    //normalize(uncompressed, decoded_size);

    size_t num_out;
    /*
    if (buffers_full == 3) {

      if (next_out == 1) {
        uncompressed = uncompressed1;
        next_out = 2;
        buffers_full--;
       // buf1 = false;
      }
      else if (next_out == 2) {
        uncompressed = uncompressed2;
        next_out = 3;
        buffers_full--;
       // buf2 = false;
      }
      else {
        uncompressed = uncompressed3;
        next_out = 1;
        buffers_full--;
        //buf3 = false;
      }
        */

    i2s_write(I2S_PORT_OUT, uncompressed, OPUS_FRAME_SIZE * sizeof(int16_t), &num_out, 0);
    //}
    
   // memset(dataIn, 0, 1275 * sizeof(uint8_t));
    //memset(uncompressed, 0, OPUS_FRAME_SIZE * sizeof(int16_t));
  }
  else {
    Serial.println("Error = " + status);
  }

}


void onSpeak() {

  //Serial.println("Speaking");

  size_t num_in;

  memset(new_opus, 0, OPUS_FRAME_SIZE * sizeof(int16_t));
  esp_err_t err = i2s_read(I2S_PORT_IN, new_opus, OPUS_FRAME_SIZE * sizeof(int16_t), &num_in, portMAX_DELAY);

  memset(compressed, 0, 1275 * sizeof(uint8_t));
  size_t len = opus_encode(opus_encoder_, new_opus, OPUS_FRAME_SIZE, compressed, 1275 * sizeof(uint8_t));
  //Serial.println("Length");
  //Serial.println(len);

  int test = radio->transmit(compressed, len);
  //Serial.println("Result");
  //Serial.println(test);

}
  /*

void onSpeak() {

    //Serial.println("2");
    size_t num_in;
    memset(new_opus, 0, OPUS_FRAME_SIZE * sizeof(int16_t));

    esp_err_t err = i2s_read(I2S_PORT_IN, new_opus, OPUS_FRAME_SIZE * sizeof(int16_t), &num_in, portMAX_DELAY);
    //Serial.println("3");
    int test = opus_encode(opus_encoder_, new_opus, OPUS_FRAME_SIZE, compressed, 1275 * sizeof(uint8_t));
    //Serial.println("4");
    
    test = opus_decode(opus_decoder_, compressed, test, new_opus, 
                OPUS_FRAME_SIZE, 0);
                //Serial.println("5");

    size_t num_out;
    i2s_write(I2S_PORT_OUT, new_opus, OPUS_FRAME_SIZE * sizeof(int16_t), &num_out, 0);
    //Serial.println("6");
}
*/

void setFlag(void) {
  receivedFlag = true;
}

void normalize(int16_t *buf, size_t length) {

  int32_t max_val = 0;

  for (int i = 0; i < length; i++) {
    if (abs(buf[i]) > max_val) {
      max_val = buf[i];
    }
  }

  int32_t target_peak = 32767;
  float gain_factor = (max_val == 0) ? 1.0f : (float)target_peak / max_val;
  
  for (int i = 0; i < length; ++i) {
    int32_t amplified = buf[i] * GAIN_FACTOR;
    if (amplified > 32767) amplified = 32767;
    if (amplified < -32768) amplified = -32768;
    buf[i] = (int16_t)amplified;
 }


  
}
