#include <AES.h>
#include <Arduino.h>
#include <Audio.h>
#include <Crypto.h>
#include <driver/i2s.h>
#include <GCM.h>
#include <opus.h>
#include <RadioLib.h>
#include <SPI.h>
#include <stdint.h>


// Universal Sample Rate
#define SAMPLE_RATE 16000

//INMP441
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

// Compression Parameters
#define AUDIO_OPUS_FRAME_MS   20   // one of 2.5, 5, 10, 20, 40, 60, 80, 100, 120
#define AUDIO_OPUS_BITRATE    16000// bit rate from 2400 to 512000
#define AUDIO_OPUS_COMPLEXITY 0    // from 0 to 10
#define OPUS_FRAME_SIZE ((SAMPLE_RATE / 1000) * AUDIO_OPUS_FRAME_MS)

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
#define LoRa_FREQUENCY 915.0F
#define LoRa_POWER 22
#define LoRa_PREAMBLE_LEN 10U
#define LoRa_SPREADFACTOR 5
#define LoRa_SYNCWORD 0xF5
#define LoRa_TCXO_VOLTAGE 1.6
#define LoRa_USE_REGULATOR_LDO false

#define GAIN_FACTOR 45

/* GCM Encryption Sizes */
#define TAG_SIZE 16
#define IV_SIZE 12
#define CIPHER_SIZE 120

Module *lora_module;

SX1262 *radio;

OpusEncoder *opus_encoder_;
OpusDecoder *opus_decoder_;

TaskHandle_t receiveTaskHandle = NULL;
TaskHandle_t speakTaskHandle = NULL;

bool deleteSpeak = false;
bool deleteReceive = false;


/* Function Declarations */
void onSpeak();
void onReceive();
void normalize(int16_t*, size_t);
void setFlag();
void encrypt(uint8_t*, uint8_t*, uint8_t*, size_t);
void decrypt();

/* Set up I2S Processor configuration */
void i2s_install() {
  const i2s_config_t i2s_config_in = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = i2s_bits_per_sample_t(16),
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),
    .intr_alloc_flags = 0,
    .dma_buf_count = 16,
    .dma_buf_len = BUFFER_SIZE,
    .use_apll = false
  };

  const i2s_config_t i2s_config_out = {
    .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_TX),  // I2S Master mode and Transmit (TX)
    .sample_rate = SAMPLE_RATE,             // Set sample rate
    .bits_per_sample = i2s_bits_per_sample_t(16),     // Set audio bit depth (16-bit)
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT, // Stereo output
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_STAND_I2S),  // I2S communication format
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,   // Interrupt level (default)
    .dma_buf_count = 16,
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

/* Compression Buffers */
int16_t *new_opus;
uint8_t *compressed;
int16_t *uncompressed;

/* Encryption Buffers */
uint8_t *encrypted;
uint8_t *decrypted;
uint8_t *nonce;

/* Packet Buffers */
uint8_t *sendPacket;
uint8_t *receivePacket;


bool isReceiving = false;
bool isSpeaking = false;

volatile bool receivedFlag = false;
uint16_t last_received = millis();

GCM<AES256> gcm;  

uint8_t key[16] = {182, 32, 42, 2, 243, 40, 201, 140, 141, 29, 199, 216, 117, 42, 43, 167};
uint8_t tag[16];

 
void setup() {
 
  /* Set up Serial Monitor */
  Serial.begin(115200);
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

            
  status = radio->startReceive();
  radio->setPacketReceivedAction(setFlag);
  Serial.println("Status");
  Serial.println(status);
              
  if (radio->setCRC(true) == RADIOLIB_ERR_NONE) {
    Serial.println("CRC enabled successfully!");
  } else {
    Serial.println("Failed to enable CRC!");
  }

  /* Create Encoder */
  int encoder_error;
  opus_encoder_ = opus_encoder_create(SAMPLE_RATE, 1, OPUS_APPLICATION_VOIP, &encoder_error);
  if (encoder_error != OPUS_OK) {
    Serial.println("ENCODE ERROR");
    Serial.println(encoder_error);
    return;
  }
  encoder_error = opus_encoder_init(opus_encoder_, SAMPLE_RATE, 1, OPUS_APPLICATION_VOIP);
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
  opus_decoder_ = opus_decoder_create(SAMPLE_RATE, 1, &decoder_error);
  if (decoder_error != OPUS_OK) {
    Serial.println("DECODE ERROR");
    Serial.println(decoder_error);
    return;
  } 

  /* Allocate Buffers */
  new_opus = (int16_t*) malloc(OPUS_FRAME_SIZE * sizeof(int16_t) * 3);
  assert(new_opus != NULL);

  compressed = (uint8_t*) malloc(1275 * sizeof(uint8_t) * 3);
  assert(compressed != NULL);

  encrypted = (uint8_t*) malloc(1275 * sizeof(uint8_t) * 3);
  assert(encrypted != NULL);

  decrypted = (uint8_t*) malloc(1275 * sizeof(uint8_t) * 3);

  nonce = (uint8_t*) malloc(IV_SIZE);
  assert(nonce != NULL);

  sendPacket = (uint8_t*) malloc(IV_SIZE + CIPHER_SIZE + TAG_SIZE);
  assert(sendPacket != NULL);

  receivePacket = (uint8_t*) malloc(IV_SIZE + CIPHER_SIZE + TAG_SIZE);
  assert(receivePacket != NULL);

  uncompressed = (int16_t*) malloc(OPUS_FRAME_SIZE * sizeof(int16_t) * 3); //WAS * 2!!!!!!!!!!!!!!
  assert(uncompressed != NULL);

  gcm.setKey(key, sizeof(key));

  Serial.print("SETUP COMPLETE\n");
  delay(100);
}
 
void loop() {

  if (deleteSpeak && speakTaskHandle != NULL) {
    while(eTaskGetState(speakTaskHandle) == eBlocked);
    vTaskDelete(speakTaskHandle);
    speakTaskHandle = NULL;
    deleteSpeak = false;
  }
  if (deleteReceive && receiveTaskHandle != NULL) {
    radio->standby();
    while(eTaskGetState(receiveTaskHandle) == eBlocked);
    vTaskDelete(receiveTaskHandle);
    receiveTaskHandle = NULL;
    radio->startReceive();
    deleteReceive = false;
  }

  if (isSpeaking == false && digitalRead(SPEAK_BUTTON) == LOW && isReceiving == false) {
      isSpeaking = true;
      Serial.println("Start speak");

      xTaskCreatePinnedToCore(
        [](void *param){
          radio->standby();
          while (digitalRead(SPEAK_BUTTON) == LOW) {
            onSpeak();
          }
          radio->startReceive();
          receivedFlag = false;
          Serial.println("Terminating Speak");
          deleteSpeak = true;
          isSpeaking = false;
          while(1);  //freeRTOS tasks should never return, so it busy waits until it is deleted by loop()
        },
        "speakTask",     // name
        8192 * 8,        // stack size in bytes
        NULL,            // param
        1,               // priority
        &speakTaskHandle,// handle
        APP_CPU_NUM      // core
      ); 
  }
 
  if (receivedFlag == true && isSpeaking == false && isReceiving == false) {

      Serial.println("Start receive");
      isReceiving = true;
      int code = xTaskCreatePinnedToCore(
        [](void *param) {

          int last_received = millis();
          Serial.println("Start task");

          do {
            if (receivedFlag) {
              last_received = millis();
              receivedFlag = false;
              onReceive();
              radio->startReceive();
            }
          }
          while (millis() - last_received < 600);

          receivedFlag = false;
          isReceiving = false;
          Serial.println("Terminating Receive");
          deleteReceive = true;
          i2s_zero_dma_buffer(I2S_PORT_OUT);
          while(1);
        },
        "receiveTask",      // name
        8192 * 8,           // stack size in bytes
        NULL,               // param
        1,                  // priority
        &receiveTaskHandle, // handle
        1                   // core
      );
  }
}

void onReceive() {

  int length = radio->getPacketLength();
  if (length != (IV_SIZE + CIPHER_SIZE + TAG_SIZE)) {
    Serial.println("Bad packet size");
    return;
  }

  int status = radio->readData(receivePacket, length);
  if (status == RADIOLIB_ERR_NONE) {

    for (int i = 0; i < IV_SIZE; i++) {
      nonce[i] = receivePacket[i];
    }
    for (int i = IV_SIZE; i < IV_SIZE + CIPHER_SIZE; i++) {
      compressed[i - IV_SIZE] = receivePacket[i];
    }
    for (int i = IV_SIZE + CIPHER_SIZE; i < IV_SIZE + CIPHER_SIZE + TAG_SIZE; i++) {
      tag[i - (IV_SIZE + CIPHER_SIZE)] = receivePacket[i];
    }

    gcm.setIV(nonce, IV_SIZE);
    gcm.decrypt(decrypted, compressed, CIPHER_SIZE);

    // Verify the authentication tag
    bool isAuthentic = gcm.checkTag(tag, TAG_SIZE);

    if (!isAuthentic) {
      Serial.println("Inauthentic Data");
      return;
    }
    
    int decoded_size1 = opus_decode(opus_decoder_, decrypted, CIPHER_SIZE / 3, uncompressed, OPUS_FRAME_SIZE, 0);
    int decoded_size2 = opus_decode(opus_decoder_, &decrypted[CIPHER_SIZE / 3], CIPHER_SIZE / 3, &uncompressed[decoded_size1], OPUS_FRAME_SIZE, 0);
    int decoded_size3 = opus_decode(opus_decoder_, &decrypted[(CIPHER_SIZE / 3) * 2], CIPHER_SIZE / 3, &uncompressed[decoded_size1 + decoded_size2], OPUS_FRAME_SIZE, 0);
    int decoded_size = decoded_size1 + decoded_size2 + decoded_size3;

    size_t num_out;

    /* Apply Gain */
    for (int i = 0; i < decoded_size; i++) {
      int32_t amplified = uncompressed[i] * GAIN_FACTOR;
      if (amplified > 32767) amplified = 32767;
      if (amplified < -32768) amplified = -32768;
      uncompressed[i] = (int16_t)amplified;
    }

    i2s_zero_dma_buffer(I2S_PORT_OUT);
    status = i2s_write(I2S_PORT_OUT, uncompressed, OPUS_FRAME_SIZE * sizeof(int16_t) * 3, &num_out, 0);
  }
  else {
    Serial.println("Error = " + status);
  }
}


void onSpeak() {

  size_t num_in;

  esp_err_t err = i2s_read(I2S_PORT_IN, new_opus, OPUS_FRAME_SIZE * sizeof(int16_t) * 3, &num_in, portMAX_DELAY);

  size_t len1 = opus_encode(opus_encoder_, &new_opus[0], OPUS_FRAME_SIZE, compressed, 1275 * sizeof(uint8_t));
  size_t len2 = opus_encode(opus_encoder_, &new_opus[OPUS_FRAME_SIZE], OPUS_FRAME_SIZE, &compressed[len1], 1275 * sizeof(uint8_t));
  size_t len3 = opus_encode(opus_encoder_, &new_opus[OPUS_FRAME_SIZE * 2], OPUS_FRAME_SIZE, &compressed[len1 + len2], 1275 * sizeof(uint8_t));
  size_t len = len1 + len2 + len3;

  encrypt(encrypted, compressed, nonce, len);

  for (int i = 0; i < IV_SIZE; i++) {
    sendPacket[i] = nonce[i];
  }
  for (int i = IV_SIZE; i < IV_SIZE + CIPHER_SIZE; i++) {
    sendPacket[i] = encrypted[i - IV_SIZE];
  }
  for (int i = IV_SIZE + CIPHER_SIZE; i < IV_SIZE + CIPHER_SIZE + TAG_SIZE; i++) {
    sendPacket[i] = tag[i - (IV_SIZE + CIPHER_SIZE)];
  }

  int ret = radio->transmit(sendPacket, IV_SIZE + CIPHER_SIZE + TAG_SIZE);
}

void encrypt(uint8_t *ciphertext, uint8_t *plaintext, uint8_t *nonce, size_t len) {

  for (int i = 0; i < 3; i++) {
    int rand = esp_random();
    int mask = 0b11111111;

    for (int j = 0; j < 4; j++) {
      nonce[(i*4) + j] = (rand>>(j*8) & mask);
    }
  }
  bool test1 = gcm.setIV(nonce, IV_SIZE);
  if (test1 == false) {
    Serial.println("IV");
  }
  gcm.encrypt(ciphertext, plaintext, len);
  gcm.computeTag(tag, TAG_SIZE);
}

void decrypt(uint8_t *decrypted, uint8_t *ciphertext, size_t len) {
  gcm.decrypt(decrypted, ciphertext, len);
}


void setFlag(void) {
  receivedFlag = true;
}

