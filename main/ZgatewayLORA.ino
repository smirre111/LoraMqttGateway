/*  
  OpenMQTTGateway  - ESP8266 or Arduino program for home automation 

   Act as a wifi or ethernet gateway between your 433mhz/infrared IR/BLE signal  and a MQTT broker 
   Send and receiving command by MQTT
 
  This gateway enables to:
 - receive MQTT data from a topic and send LORA signal corresponding to the received MQTT data
 - publish MQTT data to a different topic related to received LORA signal

    Copyright: (c)Florian ROBERT
  
    This file is part of OpenMQTTGateway.
    
    OpenMQTTGateway is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    OpenMQTTGateway is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include <LoRa.h>
#include <SPI.h>
#include <Wire.h>

#include "User_config.h"
#include "blinds.pb-c.h"
#include "config_LORA.h"

#define PB 1
#define ZgatewayLORA

#ifdef ZgatewayLORA

void createDiscoveryRadio(int senderAddress) {
  const int JSON_MSG_CALC_BUFFER = JSON_OBJECT_SIZE(64);
  StaticJsonBuffer<JSON_MSG_CALC_BUFFER> jsonBuffer;
  JsonObject& sensor = jsonBuffer.createObject();

  String sendAddrStr = String(senderAddress);

  String loraTopic = "home/OpenMQTTGateway_ESP32_LORA/LORAtoMQTT/" + sendAddrStr;

  sensor.set("name", "cover"); //name
  sensor.set("uniq_id", sendAddrStr); //unique_id
  sensor.set("dev_cla", "blind");
  sensor.set("cmd_t", "home/OpenMQTTGateway_ESP32_LORA/commands/MQTTtoLORA");

  sensor.set("stat_t", loraTopic + "/status");
  sensor.set("pos_t", loraTopic + "/position");
  sensor.set("avty_t", loraTopic + "/available");

  sensor.set("pl_avail", "online");
  sensor.set("pl_not_avail", "offline");

  sensor.set("qos", 0);
  sensor.set("ret", true);

  sensor.set("pl_open", "{address: " + sendAddrStr + ",subnet: 1, sender_address: 0, msgId: 0, cmd: \"OPEN\", message: \"test\"}");
  sensor.set("pl_cls", "{address: " + sendAddrStr + ",subnet: 1, sender_address: 0, msgId: 0, cmd: \"CLOSE\", message: \"test\"}");
  sensor.set("pl_stop", "{address: " + sendAddrStr + ",subnet: 1, sender_address: 0, msgId: 0, cmd: \"STOP\", message: \"test\"}");

  sensor.set("stat_open", "open");
  sensor.set("stat_opening", "opening");
  sensor.set("stat_stopped", "stopped");
  sensor.set("stat_closing", "closing");
  sensor.set("stat_clsd", "closed");

  sensor.set("opt", false);

  sensor.set("val_tpl", "{{ value_json['message']['state'] }}");
  sensor.set("position_template", "{{ value_json['message']['position'] }}");

  String topic = String(discovery_Topic) + "/" + "cover" + "/" + sendAddrStr + "/config";
  pub_custom_topic((char*)topic.c_str(), sensor, true);
}

void createDiscoverSensor(int senderAddress) {
  const int JSON_MSG_CALC_BUFFER = JSON_OBJECT_SIZE(64);
  StaticJsonBuffer<JSON_MSG_CALC_BUFFER> jsonBuffer;
  JsonObject& sensor = jsonBuffer.createObject();

  String sendAddrStr = String(senderAddress);

  String loraTopic = "home/OpenMQTTGateway_ESP32_LORA/LORAtoMQTT/" + sendAddrStr;

  sensor.set("name", "cover voltage"); //name
  sensor.set("uniq_id", sendAddrStr + "_voltage"); //unique_id
  // sensor.set("dev_cla", "blind");
  // sensor.set("cmd_t", "home/OpenMQTTGateway_ESP32_LORA/commands/MQTTtoLORA");

  sensor.set("stat_t", loraTopic + "/status");
  // sensor.set("pos_t", loraTopic + "/position");
  sensor.set("avty_t", loraTopic + "/available");

  sensor.set("pl_avail", "online");
  sensor.set("pl_not_avail", "offline");

  // sensor.set("qos", 0);
  // sensor.set("ret", true);

  // sensor.set("pl_open", "{address: " + sendAddrStr + ",subnet: 1, sender_address: 0, msgId: 0, cmd: \"OPEN\", message: \"test\"}");
  // sensor.set("pl_cls", "{address: " + sendAddrStr + ",subnet: 1, sender_address: 0, msgId: 0, cmd: \"CLOSE\", message: \"test\"}");
  // sensor.set("pl_stop", "{address: " + sendAddrStr + ",subnet: 1, sender_address: 0, msgId: 0, cmd: \"STOP\", message: \"test\"}");

  // sensor.set("stat_open", "open");
  // sensor.set("stat_opening", "opening");
  // sensor.set("stat_stopped", "stopped");
  // sensor.set("stat_closing", "closing");
  // sensor.set("stat_clsd", "closed");

  // sensor.set("opt", false);
  sensor.set("unit_of_meas", "V");

  sensor.set("val_tpl", "{{ value_json['message']['volt'] }}");
  // sensor.set("position_template", "{{ value_json['message']['position'] }}");

  String topic = String(discovery_Topic) + "/" + "sensor" + "/" + sendAddrStr + "/config";
  pub_custom_topic((char*)topic.c_str(), sensor, true);
}

blinds_syscmd_base_t MQTTMessageToLORACmd(const JsonObject& LORAdata) {
  blinds_syscmd_base_t cmd;
  const char* message = LORAdata["cmd"];
  if (strcmp(message, "OPEN") == 0) {
    cmd = BlindsSysCmd::SYSCMD_OPEN;
    Log.notice(F("Decoded OPEN" CR));
  } else if (strcmp(message, "CLOSE") == 0) {
    cmd = BlindsSysCmd::SYSCMD_CLOSE;
    Log.notice(F("Decoded CLOSE" CR));
  } else if (strcmp(message, "ENABLE_WIFI") == 0) {
    cmd = BlindsSysCmd::SYSCMD_ENABLE_WIFI;
    Log.notice(F("Decoded ENABLE WIFI" CR));
  } else if (strcmp(message, "DIABLE_WIFI") == 0) {
    cmd = BlindsSysCmd::SYSCMD_DISABLE_WIFI;
    Log.notice(F("Decoded DISABLE WIFI" CR));
  } else if (strcmp(message, "OTA") == 0) {
    cmd = BlindsSysCmd::SYSCMD_OTA;
    Log.notice(F("Decoded OTA" CR));
  } else if (strcmp(message, "STATUS") == 0) {
    cmd = BlindsSysCmd::SYSCMD_STATUS;
    Log.notice(F("Decoded STATUS" CR));
  } else if (strcmp(message, "STOP") == 0) {
    cmd = BlindsSysCmd::SYSCMD_STOP;
    Log.notice(F("Decoded STOP" CR));
  }

  return cmd;
}

BlndOperation MQTTMessageToPbCmd(const JsonObject& LORAdata) {
  BlndOperation cmd;
  const char* message = LORAdata["cmd"];
  if (strcmp(message, "OPEN") == 0) {
    cmd = BLND_OPERATION__CMD_OPEN;
    Log.notice(F("Decoded OPEN" CR));
  } else if (strcmp(message, "CLOSE") == 0) {
    cmd = BLND_OPERATION__CMD_CLOSE;
    Log.notice(F("Decoded CLOSE" CR));
  } else if (strcmp(message, "STOP") == 0) {
    cmd = BLND_OPERATION__CMD_STOP;
    Log.notice(F("Decoded STOP" CR));
  } else {
    cmd = BLND_OPERATION__CMD_IDLE;
    Log.notice(F("Decoded No valid message" CR));
  }

  return cmd;
}

SysOperation MQTTMessageToPbSys(const JsonObject& LORAdata) {
  SysOperation cmd;
  const char* message = LORAdata["sys"];
  if (strcmp(message, "ENABLE_WIFI") == 0) {
    cmd = SYS_OPERATION__CMD_ENABLE_WIFI;
    Log.notice(F("Decoded ENABLE WIFI" CR));
  } else if (strcmp(message, "DIABLE_WIFI") == 0) {
    cmd = SYS_OPERATION__CMD_DISABLE_WIFI;
    Log.notice(F("Decoded DISABLE WIFI" CR));
  } else if (strcmp(message, "OTA") == 0) {
    cmd = SYS_OPERATION__CMD_OTA;
    Log.notice(F("Decoded OTA" CR));
  } else if (strcmp(message, "STATUS") == 0) {
    cmd = SYS_OPERATION__CMD_STATUS;
    Log.notice(F("Decoded STATUS" CR));
  }

  return cmd;
}

const char* stateOpen{"open"};
const char* stateOpening{"opening"};
const char* stateClosing{"closing"};
const char* stateClosed{"closed"};
const char* stateStopped{"stopped"};

const char* blindsStateToString(const mqtt_blinds_state_t& state) {
  switch (state) {
    case 0:
      return stateOpen;
      break;
    case 1:
      return stateOpening;
      break;
    case 2:
      return stateClosing;
      break;
    case 3:
      return stateClosed;
      break;
    case 4:
      return stateStopped;
      break;
    default:
      return stateOpen;
  }
}

const char* blindsStatePbToString(const BlndState& state) {
  switch (state) {
    case BLND_STATE__BLINDS_MQTT_OPEN:
      return stateOpen;
      break;
    case BLND_STATE__BLINDS_MQTT_OPENING:
      return stateOpening;
      break;
    case BLND_STATE__BLINDS_MQTT_CLOSING:
      return stateClosing;
      break;
    case BLND_STATE__BLINDS_MQTT_CLOSED:
      return stateClosed;
      break;
    case BLND_STATE__BLINDS_MQTT_STOPPED:
      return stateStopped;
      break;
    default:
      return stateOpen;
  }
}

void setupLORA() {
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DI0);

  if (!LoRa.begin(LORA_BAND)) {
    Log.error(F("ZgatewayLORA setup failed!" CR));
    while (1)
      ;
  }

  LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
  LoRa.setSyncWord(LORA_SYNC_WORD);

  LoRa.receive();
  Log.notice(F("LORA_SCK: %d" CR), LORA_SCK);
  Log.notice(F("LORA_MISO: %d" CR), LORA_MISO);
  Log.notice(F("LORA_MOSI: %d" CR), LORA_MOSI);
  Log.notice(F("LORA_SS: %d" CR), LORA_SS);
  Log.notice(F("LORA_RST: %d" CR), LORA_RST);
  Log.notice(F("LORA_DI0: %d" CR), LORA_DI0);
  Log.trace(F("ZgatewayLORA setup done" CR));
}

void LORAtoMQTT() {
  LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
  LoRa.setCodingRate4(LORA_CODING_RATE);
  LoRa.setPreambleLength(LORA_PREAMBLE_LENGTH_RX);
  LoRa.setSignalBandwidth(LORA_SIGNAL_BANDWIDTH);
  LoRa.setSyncWord(LORA_SYNC_WORD);
  LoRa.enableCrc();

  int packetSize = LoRa.parsePacket();

  if (packetSize) {
    StaticJsonBuffer<JSON_MSG_BUFFER> jsonBuffer;
    JsonObject& LORAdata = jsonBuffer.createObject();
    Log.trace(F("Rcv. LORA" CR));
#  ifdef ESP32
    String taskMessage = "LORA Task running on core ";
    taskMessage = taskMessage + xPortGetCoreID();
    //trc(taskMessage);
#  endif

    uint8_t packet[256];
    // String packet;
    // packet = "";
    for (int i = 0; i < packetSize; i++) {
      packet[i] = (char)LoRa.read();
    }
    packet[packetSize] = 0;

    //packet.getBytes(cmd, length, 5);

    // Log.notice(F("Cmd:" CR));
    // for (int i = 0; i < length; i++) {
    //   Log.notice(F("Byte %d: %d" CR), i, cmd[i]);
    // }

#  if (PB == 0)
    uint8_t destAddress = uint8_t(packet[0]);
    uint8_t destSubnet = uint8_t(packet[1]);
    uint8_t senderAddress = uint8_t(packet[2]);
    uint8_t msgId = uint8_t(packet[3]);
    uint8_t length = uint8_t(packet[4]);
#  else
    // BlndResponseMessage msg = BLND_RESPONSE_MESSAGE__INIT;

    // msg.destaddress = 14;
    // msg.destsubnet = 11;
    // msg.senderaddress = 11;
    // msg.msgid = 1;

    // uint8_t *buf;                     // Buffer to store serialized data
    // unsigned len;                  // Length of serialized data

    // len = blnd_response_message__get_packed_size(&msg);

    // buf = (uint8_t *) malloc(len);

    // blnd_response_message__pack(&msg, buf);

    // free(buf);

    // #define MAX_MSG_SIZE 1024
    BlndResponseMessage* rcv_message;
    // Read packed message from standard-input.
    // uint8_t rcv_buf[MAX_MSG_SIZE];

    // strcpy((char *)rcv_buf, "Test the buffer");
    // // Read binary data
    // size_t msg_len = 10; //read_buffer (MAX_MSG_SIZE, rcv_buf);

    rcv_message = blnd_response_message__unpack(NULL, packetSize, packet);

    if (rcv_message == NULL) {
      ESP_LOGE(TAG, "Could not read protobuf");
      return;
    }

#  endif

    // Log.notice(F("Packet length: %d" CR), length);
    Log.notice(F("Received length: %d" CR), packetSize - 5);

    Log.notice(F("Payload:" CR));
    for (int i = 0; i < packetSize; i++) {
      Log.notice(F("Byte %d: %d" CR), i, packet[i]);
    }

    char LORAmessage[64];
    JsonObject& LORAsubdata = jsonBuffer.createObject();
    bool jsonMessage = true;

#  if (PB == 0)
    if (char(packet[5]) == char('l')) {
      LORAsubdata.set("log", (char*)(packet + 6));
    } else if (char(packet[5]) == char('s')) {
      byte state = packet[6];
      int* adcReading = (int*)(packet + 7);
      float voltage = *adcReading * adcRatio;
      Log.notice(F("State: %d" CR), state);
      Log.notice(F("Voltage: %d" CR), voltage);
      LORAsubdata.set("state", blindsStateToString(mqtt_blinds_state_t(state)));
      LORAsubdata.set("volt", voltage);
    } else if (char(packet[5]) == char('a')) {
      if (char(packet[6]) == char('1')) {
        strcpy(LORAmessage, onlineString);
      } else {
        strcpy(LORAmessage, offlineString);
      }
      jsonMessage = false;
    }
#  else
    switch (rcv_message->proto_case) {
      case BLND_RESPONSE_MESSAGE__PROTO__NOT_SET: {
        Log.notice(F("Proto not set:" CR));
      } break;
      case BLND_RESPONSE_MESSAGE__PROTO_STATE: {
        BlndStatus* status = rcv_message->state;
        float voltage = status->voltage;
        int position = int(status->position);
        BlndState state = status->state;
        LORAsubdata.set("state", blindsStatePbToString(state));
        LORAsubdata.set("volt", voltage);
        LORAsubdata.set("position", position);
        // blnd_status__free_unpacked(status, NULL);

      } break;
      case BLND_RESPONSE_MESSAGE__PROTO_LOGGING: {
        BlndLogging* logmsg = rcv_message->logging;
        LORAsubdata.set("log", logmsg->logmsg);
        // blnd_logging__free_unpacked(logmsg, NULL);

      } break;
      case BLND_RESPONSE_MESSAGE__PROTO_AVAIL: {
        BlndAvailable* avail = rcv_message->avail;
        if (avail->available == true) {
          strcpy(LORAmessage, onlineString);
        } else {
          strcpy(LORAmessage, offlineString);
        }
        // blnd_available__free_unpacked(avail, NULL);
        jsonMessage = false;

      } break;
    }
#  endif

    const String statusTopic = "status";
    const String availableTopic = "available";
    const String positionTopic = "position";

#  if (PB == 0)
    String mac_address(senderAddress);
#  else
    String mac_address(rcv_message->senderaddress);

#  endif

    String mactopic{};
    if (jsonMessage) {
      mactopic = subjectLORAtoMQTT + String("/") + mac_address + String("/") + statusTopic;
    } else {
      mactopic = subjectLORAtoMQTT + String("/") + mac_address + String("/") + availableTopic;
    }
    String mactopicPos{};
    mactopicPos = subjectLORAtoMQTT + String("/") + mac_address + String("/") + positionTopic;

#  if (PB == 0)
    LORAsubdata.set("destAddress", (int)destAddress);
    LORAsubdata.set("destSubnet", (int)destSubnet);
    LORAsubdata.set("senderAddress", (int)senderAddress);
    LORAsubdata.set("msgId", (int)msgId);
#  else
    LORAsubdata.set("destAddress", (int)rcv_message->destaddress);
    LORAsubdata.set("destSubnet", (int)rcv_message->destsubnet);
    LORAsubdata.set("senderAddress", (int)rcv_message->senderaddress);
    LORAsubdata.set("msgId", (int)rcv_message->msgid);

    //blnd_response_message__free_unpacked(rcv_message, NULL);

#  endif

    LORAdata.set("rssi", (int)LoRa.packetRssi());
    LORAdata.set("snr", (float)LoRa.packetSnr());
    LORAdata.set("pferror", (float)LoRa.packetFrequencyError());
    LORAdata.set("packetSize", (int)packetSize);
    LORAdata.set("message", LORAsubdata); //(char*)packet);

    float snr = LoRa.packetSnr();
    if (snr > -10) {
      if (jsonMessage) {
        //pub(subjectLORAtoMQTT, LORAdata);
        pub((char*)mactopic.c_str(), LORAdata);
        pub((char*)mactopicPos.c_str(), LORAdata);
      } else {
        //pub(subjectLORAtoMQTT, LORAdata);
        createDiscoveryRadio((int)rcv_message->senderaddress);
        createDiscoverSensor((int)rcv_message->senderaddress);

        pub((char*)mactopic.c_str(), LORAmessage);
      }

      if (repeatLORAwMQTT) {
        Log.trace(F("Pub LORA for rpt" CR));
        if (jsonMessage) {
          //pub(subjectLORAtoMQTT, LORAdata);
          pub((char*)mactopic.c_str(), LORAdata);
          pub((char*)mactopicPos.c_str(), LORAdata);

        } else {
          //pub(subjectLORAtoMQTT, LORAdata);
          createDiscoveryRadio((int)rcv_message->senderaddress);
          createDiscoverSensor((int)rcv_message->senderaddress);

          pub((char*)mactopic.c_str(), LORAmessage);
        }
      }
    } else {
      Log.notice(F("SNR too low for valid message:" CR));
    }

#  if (PB == 0)

#  else

    blnd_response_message__free_unpacked(rcv_message, NULL);

#  endif
  }
}

// LoRa.write(packet.destAddress);                   // add destination address
// LoRa.write(packet.destSubnet);                    // add destination subnet
// LoRa.write(packet.senderAddress);                 // add senderAddress address
// LoRa.write(packet.msgId);                         // add message ID
// LoRa.write(packet.payloadLength);                 // add payload length
// LoRa.write(packet.payload, packet.payloadLength); // add payload

#  ifdef jsonReceiving
void MQTTtoLORA(char* topicOri, JsonObject& LORAdata) { // json object decoding
  if (cmpToMainTopic(topicOri, subjectMQTTtoLORA)) {
    //uint8_t deviceAddress = getAddress(topicOri, subjectMQTTtoLORA);

    Log.trace(F("MQTTtoLORA json" CR));

    uint8_t destAddress = LORAdata["address"];
    uint8_t destSubnet = LORAdata["subnet"];
    uint8_t senderAddress = 0;
    uint8_t msgId = LORAdata["msgId"];
    //const char* cmdString = LORAdata["cmd"];
    //uint8_t cmd = MQTTMessageToLORACmd(LORAdata);
    uint8_t length = 1;
    const char* message = LORAdata["message"];

#    if (PB == 1)

    BlndOperation op_init = BLND_OPERATION__CMD_IDLE;
    // BlndOperationMessage op_message = BLND_OPERATION_MESSAGE__INIT;
    // op_message.destaddress = destAddress;
    // op_message.destsubnet = destSubnet;
    // op_message.senderaddress = senderAddress;
    // op_message.msgid = msgId;

    BlndOperationMessage op_message = {PROTOBUF_C_MESSAGE_INIT(&blnd_operation_message__descriptor),
                                       destAddress,
                                       destSubnet,
                                       senderAddress,
                                       msgId,
                                       BLND_OPERATION_MESSAGE__CMD__NOT_SET,
                                       op_init};

    if (LORAdata.containsKey("cmd")) {
      BlndOperation operation = MQTTMessageToPbCmd(LORAdata);
      op_message.cmd_case = BLND_OPERATION_MESSAGE__CMD_OPERATION;
      op_message.operation = operation;
    } 
    else if (LORAdata.containsKey("sys")) {
      SysOperation sysOp = MQTTMessageToPbSys(LORAdata);
      op_message.cmd_case = BLND_OPERATION_MESSAGE__CMD_SYSOP;
      op_message.sysop = sysOp;
    } 
    else if (LORAdata.containsKey("addressing")) {
      AddressUpdate addrUpd = ADDRESS_UPDATE__INIT;
      addrUpd.addr = LORAdata["addressing"]["addr"];
      addrUpd.subnt = LORAdata["addressing"]["subnet"];
      Log.notice(F("New address: %d" CR), addrUpd.addr);
      Log.notice(F("New subnet: %d" CR), addrUpd.subnt);
      op_message.cmd_case = BLND_OPERATION_MESSAGE__CMD_ADDR;
      op_message.addr =  {&addrUpd};
    }
    else if (LORAdata.containsKey("timing")) {
      TimeUpdate timeUpd = TIME_UPDATE__INIT;
      timeUpd.opentime = LORAdata["timing"]["open_time"];
      timeUpd.closetime = LORAdata["timing"]["close_time"];
      op_message.cmd_case = BLND_OPERATION_MESSAGE__CMD_TIME;
      op_message.time = {&timeUpd};
    }



    uint8_t* txBuf;
    unsigned len;
    len = blnd_operation_message__get_packed_size(&op_message);
    txBuf = new uint8_t[len];
    blnd_operation_message__pack(&op_message, txBuf);

#    else

    uint8_t payload[7];
    payload[0] = uint8_t(destAddress);
    payload[1] = uint8_t(destSubnet);
    payload[2] = uint8_t(senderAddress);
    payload[3] = uint8_t(msgId);
    payload[4] = uint8_t(length);
    payload[5] = uint8_t(cmd);
    payload[6] = '\0';

    Log.notice(F("Message to send: : %s" CR), payload);

#    endif

    int txPower = LORAdata["txpower"] | LORA_TX_POWER;
    int spreadingFactor = LORAdata["spreadingfactor"] | LORA_SPREADING_FACTOR;
    long int frequency = LORAdata["frequency "] | LORA_BAND;
    long int signalBandwidth = LORAdata["signalbandwidth"] | LORA_SIGNAL_BANDWIDTH;
    int codingRateDenominator = LORAdata["codingrate"] | LORA_CODING_RATE;
    int preambleLength = LORAdata["preamblelength"] | LORA_PREAMBLE_LENGTH;
    byte syncWord = LORAdata["syncword"] | LORA_SYNC_WORD;
    bool Crc = LORAdata["enablecrc"] | DEFAULT_CRC;

    if (message) {
      LoRa.setTxPower(txPower);
      LoRa.setFrequency(frequency);
      LoRa.setSpreadingFactor(spreadingFactor);
      LoRa.setSignalBandwidth(signalBandwidth);
      LoRa.setCodingRate4(codingRateDenominator);
      LoRa.setPreambleLength(preambleLength);
      LoRa.setSyncWord(syncWord);
      if (Crc) {
        LoRa.enableCrc();
      }

#    if (PB == 1)
      LoRa.beginPacket();
      LoRa.write(txBuf, len);
      //LoRa.print(message);
      LoRa.endPacket();
#    else
      LoRa.beginPacket();
      LoRa.write(payload, 6);
      //LoRa.print(message);
      LoRa.endPacket();
#    endif

      Log.trace(F("MQTTtoLORA OK" CR));
      pub(subjectGTWLORAtoMQTT, LORAdata); // we acknowledge the sending by publishing the value to an acknowledgement topic, for the moment even if it is a signal repetition we acknowledge also
    } else {
      Log.error(F("MQTTtoLORA Fail json" CR));
    }
  }
}
#  endif
#  ifdef simpleReceiving
void MQTTtoLORA(char* topicOri, char* LORAdata) { // json object decoding
  if (cmpToMainTopic(topicOri, subjectMQTTtoLORA)) {
    LoRa.beginPacket();
    LoRa.print(LORAdata);
    LoRa.endPacket();
    Log.notice(F("MQTTtoLORA OK" CR));
    pub(subjectGTWLORAtoMQTT, LORAdata); // we acknowledge the sending by publishing the value to an acknowledgement topic, for the moment even if it is a signal repetition we acknowledge also
  }
}
#  endif
#endif
