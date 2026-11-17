#include "bt_telemetry.h"
#include <BluetoothSerial.h>
#include <cstring>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED) || !defined(CONFIG_BT_SPP_ENABLED)
#error "Bluetooth Classic SPP requires original ESP32 and Bluedroid"
#endif
namespace {
BluetoothSerial SerialBT;
constexpr size_t FRAME_BYTES=2+sizeof(Telemetry)+1;
constexpr UBaseType_t QUEUE_FRAMES=12;
struct Frame { uint8_t bytes[FRAME_BYTES]; };
QueueHandle_t txQueue=nullptr;
SemaphoreHandle_t txLock=nullptr;
TaskHandle_t txTask=nullptr;
volatile uint32_t queued=0,sent=0,dropped=0,failed=0;
void txWorker(void *) {
  Frame frame{};
  for(;;){
    if(xQueueReceive(txQueue,&frame,portMAX_DELAY)!=pdTRUE)continue;
    if(!SerialBT.hasClient()){++dropped;continue;}
    // BluetoothSerial.write can wait for SPP buffers. Run off the PID loop.
    if(xSemaphoreTake(txLock,portMAX_DELAY)==pdTRUE){
      const size_t n=SerialBT.write(frame.bytes,sizeof(frame.bytes));
      xSemaphoreGive(txLock);
      if(n==sizeof(frame.bytes))++sent; // Accepted by SPP, NOT delivered to PC.
      else ++failed;
    }
  }
}
}
bool btBegin(const char *name){
  if(!SerialBT.begin(name))return false;
  txQueue=xQueueCreate(QUEUE_FRAMES,sizeof(Frame));
  txLock=xSemaphoreCreateMutex();
  if(!txQueue||!txLock){
    if(txQueue)vQueueDelete(txQueue);
    if(txLock)vSemaphoreDelete(txLock);
    txQueue=nullptr;txLock=nullptr;SerialBT.end();return false;
  }
  if(xTaskCreatePinnedToCore(txWorker,"spp_telemetry",3072,nullptr,1,&txTask,0)!=pdPASS){
    vQueueDelete(txQueue);vSemaphoreDelete(txLock);
    txQueue=nullptr;txLock=nullptr;SerialBT.end();return false;
  }
  return true;
}
bool btConnected(){return SerialBT.hasClient();}
void btSendLine(const String &line){
  if(!btConnected()||!txLock)return;
  String msg=line;msg+='\n';
  // Text replies are rare and explicitly requested; serialize whole writes
  // with the binary worker so a line never corrupts a 72-byte frame.
  if(xSemaphoreTake(txLock,pdMS_TO_TICKS(100))==pdTRUE){
    SerialBT.write(reinterpret_cast<const uint8_t *>(msg.c_str()),msg.length());
    xSemaphoreGive(txLock);
  }
}
bool btSendTelemetry(const Telemetry &sample){
  if(!btConnected()||!txQueue)return false;
  Frame frame{};
  frame.bytes[0]=0xAA;frame.bytes[1]=0x55;
  memcpy(frame.bytes+2,&sample,sizeof(sample));
  uint8_t check=0;
  for(size_t i=0;i<sizeof(sample);++i)check^=frame.bytes[2+i];
  frame.bytes[FRAME_BYTES-1]=check;
  if(xQueueSend(txQueue,&frame,0)!=pdTRUE){++dropped;return false;}
  ++queued;return true;
}
BtTxStats btGetTxStats(){
  BtTxStats s{};
  s.queued=queued;s.sent=sent;s.dropped=dropped;s.failed=failed;
  s.pending=txQueue?uxQueueMessagesWaiting(txQueue):0;
  return s;
}
void btDropPendingTelemetry(){
  if(txQueue){
    const uint32_t n=uxQueueMessagesWaiting(txQueue);
    xQueueReset(txQueue);dropped+=n;
  }
}
void btPollCommands(BtCommandHandler handler){
  static String line;
  uint16_t n=0;
  while(SerialBT.available()&&n++<128){
    char c=(char)SerialBT.read();
    if(c=='\n'||c=='\r'){
      if(line.length()&&handler)handler(line);
      line="";
    } else if(c>=32&&c<=126&&line.length()<80)line+=c;
    else if(line.length()>=80)line="";
  }
}
