package m6e

import (
        "errors"
	"log"

	"github.com/tarm/serial"
)

type opcode byte

const (
  version opcode = 0x03
  setBaudRate opcode = 0x06
  readTagIDSingle opcode = 0x21
  readTagIDMultiple opcode = 0x22
  writeTagID opcode = 0x23
  writeTagData opcode = 0x24
  killTag opcode = 0x26
  readTagData opcode = 0x28
  clearTagIDBuffer opcode = 0x2a
  multiProtocolTagOp opcode = 0x2f
  getReadTxPower opcode = 0x62
  getWriteTxPower opcode = 0x64
  getPowerMode opcode = 0x68
  getReaderOptionalParams opcode = 0x6a
  getProtocolParam opcode = 0x68
  setAntennaPort opcode = 0x91
  setTagProtocol opcode = 0x93
  setReadTxPower opcode = 0x92
  setWriteTxPower opcode = 0x94
  setRegion opcode = 0x97
  setReaderOptionalParams opcode = 0x9a
  setProtocolParam opcode = 0x9B
)

var ErrCmdResponseTimeout = errors.New("Command reponse timeout")
var ErrCorruptResponse = errors.New("Corrupt response")
var ErrWrongOpcodeResponse = errors.New("Wrong opcode response")
var ErrUnknownOpcode = errors.New("Unknown opcode")

/*#define RESPONSE_IS_TEMPERATURE 5
#define RESPONSE_IS_KEEPALIVE 6
#define RESPONSE_IS_TEMPTHROTTLE 7
#define RESPONSE_IS_TAGFOUND 8
#define RESPONSE_IS_NOTAGFOUND 9
#define RESPONSE_IS_UNKNOWN 10
#define RESPONSE_SUCCESS 11
#define RESPONSE_FAIL 12*/

type Region byte

const (
  RegionIndia Region = 0x04
  RegionJapan Region = 0x05
  RegionChina Region = 0x06
  RegionEurope Region = 0x08
  RegionKorea Region = 0x09
  RegionAustralia Region = 0x0B
  RegionNewZealand Region = 0x0C
  RegionNorthAmerica Region = 0x0D
  RegionOpen Region = 0xFF
)

type M6E struct {
  port *serial.Port
  head uint
  printDebug bool
  commandTimeoutMs uint
}

func New(cfg *serial.Config) (*M6E, error) {
    cfg.ReadTimeout = 1
    port, err := serial.OpenPort(cfg)
    if err != nil {
        return nil, err
    }
    return &M6E{
        port: port,
        head: 0,
        printDebug: false,
        commandTimeoutMs: 3000,
    }, nil
}

func (m *M6E) EnableDebugging() {
    m.printDebug = true
}

func (m *M6E) DisableDebugging() {
    m.printDebug = false
}

func (m *M6E) SetBaud(baudRate int) {
}

func (m *M6E) GetVersion() {
}

func (m *M6E) SetReadPower(powerSetting uint) {
}

func (m *M6E) GetReadPower() {
}

func (m *M6E) SetWritePower(powerSetting uint) {
}

func (m *M6E) GetWritePower() {
}

func (m *M6E) SetRegion(reg Region) {
}

func (m *M6E) SetAntennaPort() {
}

func (m *M6E) SetAntennaSearchList() {
}

func (m *M6E) SetTagProtocol(protocol byte) {
// default 0x05
}

// StartReading disables filtering and start reading continuously
func (m *M6E) StartReading() {
}

// StopReading stops continuous read. Give 1000 to 2000ms for the module to stop reading.
func (m *M6E) StopReading() {
}

func (m *M6E) sendMessage(oc opcode, data []byte, waitForResponse bool) error {
  txbuf := append([]byte{
      0xFF,
      byte(len(data)),
      byte(oc),
  }, data...)
  crc := m.calculateCRC(txbuf[1:])
  txbuf = append(txbuf, byte((crc >> 8) & 0xff))
  txbuf = append(txbuf, byte(crc & 0xff))
  if m.printDebug {
    log.Println(txbuf)
  }

  //Remove anything in the incoming buffer
  //TODO this is a bad idea if we are constantly readings tags
//  while (_nanoSerial->available())
//    _nanoSerial->read();

  _, err := m.port.Write(txbuf)
  if err != nil {
    return err
  }

  if !waitForResponse {
    return nil
  }

  return nil
}

/*
  //Wait for response with timeout
  uint32_t startTime = millis();
  while (_nanoSerial->available() == false)
  {
    if (millis() - startTime > timeOut)
    {
      if (_printDebug == true)
        _debugSerial->println(F("Time out 1: No response from module"));
      msg[0] = ERROR_COMMAND_RESPONSE_TIMEOUT;
      return;
    }
    delay(1);
  }

  // Layout of response in data array:
  // [0] [1] [2] [3]      [4]      [5] [6]  ... [LEN+4] [LEN+5] [LEN+6]
  // FF  LEN OP  STATUSHI STATUSLO xx  xx   ... xx      CRCHI   CRCLO
  messageLength = MAX_MSG_SIZE - 1; //Make the max length for now, adjust it when the actual len comes in
  uint8_t spot = 0;
  while (spot < messageLength)
  {
    if (millis() - startTime > timeOut)
    {
      if (_printDebug == true)
        _debugSerial->println(F("Time out 2: Incomplete response"));

      msg[0] = ERROR_COMMAND_RESPONSE_TIMEOUT;
      return;
    }

    if (_nanoSerial->available())
    {
      msg[spot] = _nanoSerial->read();

      if (spot == 1)                //Grab the length of this response (spot 1)
        messageLength = msg[1] + 7; //Actual length of response is ? + 7 for extra stuff (header, Length, opcode, 2 status bytes, ..., 2 bytes CRC = 7)

      spot++;

      //There's a case were we miss the end of one message and spill into another message.
      //We don't want spot pointing at an illegal spot in the array
      spot %= MAX_MSG_SIZE; //Wrap condition
    }
  }

  //Used for debugging: Does the user want us to print the command to serial port?
  if (_printDebug == true)
  {
    _debugSerial->print(F("response: "));
    printMessageArray();
  }

  //Check CRC
  crc = calculateCRC(&msg[1], messageLength - 3); //Remove header, remove 2 crc bytes
  if ((msg[messageLength - 2] != (crc >> 8)) || (msg[messageLength - 1] != (crc & 0xFF)))
  {
    msg[0] = ERROR_CORRUPT_RESPONSE;
    if (_printDebug == true)
      _debugSerial->println(F("Corrupt response"));
    return;
  }

  //If crc is ok, check that opcode matches (did we get a response to the command we sent or a different one?)
  if (msg[2] != opcode)
  {
    msg[0] = ERROR_WRONG_OPCODE_RESPONSE;
    if (_printDebug == true)
      _debugSerial->println(F("Wrong opcode response"));
    return;
  }

  //If everything is ok, load all ok into msg array
  msg[0] = ALL_GOOD;
}
*/
//Comes from serial_reader_l3.c
//ThingMagic-mutated CRC used for messages.
//Notably, not a CCITT CRC-16, though it looks close.
var crcTable = [16]uint16{
        0x0000,
        0x1021,
        0x2042,
        0x3063,
        0x4084,
        0x50a5,
        0x60c6,
        0x70e7,
        0x8108,
        0x9129,
        0xa14a,
        0xb16b,
        0xc18c,
        0xd1ad,
        0xe1ce,
        0xf1ef,
}

func (m *M6E) calculateCRC(buf []byte) uint16 {
    var crc uint16 = 0xffff

    for i := 0; i < len(buf); i++ {
      crc = ((crc << 4) | (uint16(buf[i]) >> 4)) ^ crcTable[crc >> 12];
      crc = ((crc << 4) | (uint16(buf[i]) & 0x0F)) ^ crcTable[crc >> 12];
    }

  return crc;
}

/*
  void enableReadFilter(void);
  void disableReadFilter(void);

  void setReaderConfiguration(uint8_t option1, uint8_t option2);
  void getOptionalParameters(uint8_t option1, uint8_t option2);
  void setProtocolParameters(void);
  void getProtocolParameters(uint8_t option1, uint8_t option2);

  uint8_t parseResponse(void);

  uint8_t getTagEPCBytes(void);   //Pull number of EPC data bytes from record response.
  uint8_t getTagDataBytes(void);  //Pull number of tag data bytes from record response. Often zero.
  uint16_t getTagTimestamp(void); //Pull timestamp value from full record response
  uint32_t getTagFreq(void);      //Pull Freq value from full record response
  int8_t getTagRSSI(void);        //Pull RSSI value from full record response

  bool check(void);

  uint8_t readTagEPC(uint8_t *epc, uint8_t &epcLength, uint16_t timeOut = COMMAND_TIME_OUT);
  uint8_t writeTagEPC(char *newID, uint8_t newIDLength, uint16_t timeOut = COMMAND_TIME_OUT);

  uint8_t readData(uint8_t bank, uint32_t address, uint8_t *dataRead, uint8_t &dataLengthRead, uint16_t timeOut = COMMAND_TIME_OUT);
  uint8_t writeData(uint8_t bank, uint32_t address, uint8_t *dataToRecord, uint8_t dataLengthToRecord, uint16_t timeOut = COMMAND_TIME_OUT);

  uint8_t readUserData(uint8_t *userData, uint8_t &userDataLength, uint16_t timeOut = COMMAND_TIME_OUT);
  uint8_t writeUserData(uint8_t *userData, uint8_t userDataLength, uint16_t timeOut = COMMAND_TIME_OUT);

  uint8_t readKillPW(uint8_t *password, uint8_t &passwordLength, uint16_t timeOut = COMMAND_TIME_OUT);
  uint8_t writeKillPW(uint8_t *password, uint8_t passwordLength, uint16_t timeOut = COMMAND_TIME_OUT);

  uint8_t readAccessPW(uint8_t *password, uint8_t &passwordLength, uint16_t timeOut = COMMAND_TIME_OUT);
  uint8_t writeAccessPW(uint8_t *password, uint8_t passwordLength, uint16_t timeOut = COMMAND_TIME_OUT);

  uint8_t readTID(uint8_t *tid, uint8_t &tidLength, uint16_t timeOut = COMMAND_TIME_OUT);
  uint8_t readUID(uint8_t *tid, uint8_t &tidLength, uint16_t timeOut = COMMAND_TIME_OUT);

  uint8_t killTag(uint8_t *password, uint8_t passwordLength, uint16_t timeOut = COMMAND_TIME_OUT);

  void sendMessage(uint8_t opcode, uint8_t *data = 0, uint8_t size = 0, uint16_t timeOut = COMMAND_TIME_OUT, boolean waitForResponse = true);
  void sendCommand(uint16_t timeOut = COMMAND_TIME_OUT, boolean waitForResponse = true);

  void printMessageArray(void);
};
*/
