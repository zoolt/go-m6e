package m6e

import (
	"errors"
	"log"
	"time"

	"github.com/tarm/serial"
)

type opcode byte

const (
	version                 opcode = 0x03
	setBaudRate             opcode = 0x06
	readTagIDSingle         opcode = 0x21
	readTagIDMultiple       opcode = 0x22
	writeTagID              opcode = 0x23
	writeTagData            opcode = 0x24
	killTag                 opcode = 0x26
	readTagData             opcode = 0x28
	clearTagIDBuffer        opcode = 0x2a
	multiProtocolTagOp      opcode = 0x2f
	getReadTxPower          opcode = 0x62
	getWriteTxPower         opcode = 0x64
	getPowerMode            opcode = 0x68
	getReaderOptionalParams opcode = 0x6a
	getProtocolParam        opcode = 0x68
	setAntennaPort          opcode = 0x91
	setTagProtocol          opcode = 0x93
	setReadTxPower          opcode = 0x92
	setWriteTxPower         opcode = 0x94
	setRegion               opcode = 0x97
	setReaderOptionalParams opcode = 0x9a
	setProtocolParam        opcode = 0x9B
)

type TagProtocol byte

const (
	TagProtocolNone            TagProtocol = 0x00
	TagProtocolISO180006B      TagProtocol = 0x03
	TagProtocolGEN2            TagProtocol = 0x05
	TagProtocolISO180006BUCODE TagProtocol = 0x06
	TagProtocolIPX64           TagProtocol = 0x07
	TagProtocolIPX256          TagProtocol = 0x08
	TagProtocolATA             TagProtocol = 0x1D
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
	RegionIndia        Region = 0x04
	RegionJapan        Region = 0x05
	RegionChina        Region = 0x06
	RegionEurope       Region = 0x08
	RegionKorea        Region = 0x09
	RegionAustralia    Region = 0x0B
	RegionNewZealand   Region = 0x0C
	RegionNorthAmerica Region = 0x0D
	RegionOpen         Region = 0xFF
)

type M6E struct {
	port       *serial.Port
	head       uint
	printDebug bool
}

func New(cfg *serial.Config) (*M6E, error) {
	cfg.ReadTimeout = 3000 * time.Millisecond
	port, err := serial.OpenPort(cfg)
	if err != nil {
		return nil, err
	}
	return &M6E{
		port:       port,
		head:       0,
		printDebug: false,
	}, nil
}

func (m *M6E) EnableDebugging() {
	m.printDebug = true
}

func (m *M6E) DisableDebugging() {
	m.printDebug = false
}

//func (m *M6E) SetBaud(baudRate int) {
//}

func (m *M6E) GetVersion() error {
	_, err := m.sendMessage(version, nil, true)
	return err
}

// SetReadPower sets the read-power, enter a value between 1 and 27 (value in dBm)
func (m *M6E) SetReadPower(powerSetting float32) error {
	val := uint16(powerSetting * 100)
	if val > 2700 {
		val = 2700
	}
	_, err := m.sendMessage(setReadTxPower, []byte{byte(val >> 8), byte(val & 0xff)}, true)
	return err
}

func (m *M6E) GetReadPower() (float32, error) {
	power, err := m.sendMessage(getReadTxPower, []byte{0x00}, true)
	if err != nil {
		return 0, err
	}
	if len(power) < 3 {
		return 0, ErrWrongOpcodeResponse
	}
	return float32(uint16(power[1])<<8|uint16(power[2])) / 100, nil
}

// SetReadPower sets the write-power, enter a value between 1 and 27 (value in dBm)
func (m *M6E) SetWritePower(powerSetting float32) error {
	val := uint16(powerSetting * 100)
	if val > 2700 {
		val = 2700
	}
	_, err := m.sendMessage(setWriteTxPower, []byte{byte(val >> 8), byte(val & 0xff)}, true)
	return err
}

func (m *M6E) GetWritePower() (float32, error) {
	power, err := m.sendMessage(getWriteTxPower, []byte{0x00}, true)
	if err != nil {
		return 0, err
	}
	if len(power) < 3 {
		return 0, ErrWrongOpcodeResponse
	}
	return float32(uint16(power[1])<<8|uint16(power[2])) / 100, nil
}

func (m *M6E) SetRegion(reg Region) error {
	_, err := m.sendMessage(setRegion, []byte{byte(reg)}, true)
	return err
}

func (m *M6E) SetAntennaPort() error {
	_, err := m.sendMessage(setAntennaPort, []byte{0x01, 0x01}, true)
	return err
}

//func (m *M6E) SetAntennaSearchList() {
//}

func (m *M6E) SetTagProtocol(protocol TagProtocol) error {
	_, err := m.sendMessage(setTagProtocol, []byte{0x0, byte(protocol)}, true)
	return err
}

func (m *M6E) EnableReadFilter() error {
	_, err := m.sendMessage(setReaderOptionalParams, []byte{0x0C, 0x01}, true)
	return err
}

func (m *M6E) DisableReadFilter() error {
	_, err := m.sendMessage(setReaderOptionalParams, []byte{0x0C, 0x00}, true)
	return err
}

// StartReading disables filtering and start reading continuously
func (m *M6E) StartReading() error {
	err := m.DisableReadFilter()
	if err != nil {
		return err
	}

	//This blob was found by using the 'Transport Logs' option from the Universal Reader Assistant
	//And connecting the Nano eval kit from Thing Magic to the URA
	//A lot of it has been deciphered but it's easier and faster just to pass a blob than to
	//assemble every option and sub-opcode.
	configBlob := []byte{
		0x00, 0x00, //Timeout should be zero for true continuous reading
		0x01,                    // TM Option 1, for continuous reading
		byte(readTagIDMultiple), // sub command opcode
		0x00, 0x00,              // search flags
		byte(TagProtocolGEN2), // protocol ID
		0x07, 0x22, 0x10, 0x00, 0x1B, 0x03, 0xE8, 0x01, 0xFF,
	}
	_, err = m.sendMessage(multiProtocolTagOp, configBlob, true)
	return err
}

// StopReading stops continuous read. Give 1000 to 2000ms for the module to stop reading.
func (m *M6E) StopReading() error {
	_, err := m.sendMessage(multiProtocolTagOp, []byte{0x00, 0x00, 0x02}, false)
	return err
}

type response struct {
	Opcode opcode
	Status int16
	Data   []byte
}

func (m *M6E) sendMessage(oc opcode, data []byte, waitForResponse bool) ([]byte, error) {
	txbuf := append([]byte{
		0xFF,
		byte(len(data)),
		byte(oc),
	}, data...)
	crc := m.calculateCRC(txbuf[1:])
	txbuf = append(txbuf, byte((crc>>8)&0xff))
	txbuf = append(txbuf, byte(crc&0xff))
	if m.printDebug {
		log.Printf("TX: % x", txbuf)
	}

	m.port.Flush()

	_, err := m.port.Write(txbuf)
	if err != nil {
		return nil, err
	}

	if !waitForResponse {
		return nil, nil
	}

	res, err := m.readResponse()
	if err != nil {
		return nil, err
	}
	if res.Opcode != oc {
		return nil, ErrWrongOpcodeResponse
	}
	return res.Data, nil
}

type ResponseMessage struct {
	Status int
	Data   []byte
}

const (
	ResponseTemperature = iota
	ResponseKeepAlive
	ResponseTempThrottle
	ResponseTagFound
	ResponseNoTagFound
	ResponseUnknown
)

func (m *M6E) ReadMessage() (*ResponseMessage, error) {
	res, err := m.readResponse()
	if err != nil {
		return nil, err
	}
	if res.Opcode != readTagIDMultiple {
		return nil, ErrWrongOpcodeResponse
	}
	ret := &ResponseMessage{Status: ResponseUnknown}
	if len(res.Data) == 0 {
		if res.Status == 0x0400 {
			ret.Status = ResponseKeepAlive
		} else if res.Status == 0x0504 {
			ret.Status = ResponseTempThrottle
		}
	} else if len(res.Data) == 0x0a {
		ret.Status = ResponseTemperature
		ret.Data = res.Data
	} else if len(res.Data) != 0x08 {
		ret.Status = ResponseTagFound
		ret.Data = res.Data
	}
	return ret, nil
}

func (m *M6E) readResponse() (*response, error) {
	// Layout of response in data array:
	// [0] [1] [2] [3]      [4]      [5] [6]  ... [LEN+4] [LEN+5] [LEN+6]
	// FF  LEN OP  STATUSHI STATUSLO xx  xx   ... xx      CRCHI   CRCLO
	b := make([]byte, 1)
	var rxbuf []byte
	toRecv := 6
	for len(rxbuf) < toRecv {
		n, err := m.port.Read(b)
		if n > 0 {
			if len(rxbuf) == 0 && b[0] != 0xff {
				continue
			}
			rxbuf = append(rxbuf, b...)
			if len(rxbuf) == 2 {
				toRecv = int(b[0]) + 7
			}
		}
		if err != nil {
			log.Printf("Read error after %d/%d bytes: %w", len(rxbuf), toRecv, err)
			return nil, err
		}
	}

	if m.printDebug {
		log.Printf("RX: % x", rxbuf)
	}
	rxcrc := m.calculateCRC(rxbuf[1 : len(rxbuf)-2])
	if rxbuf[len(rxbuf)-2] != byte(rxcrc>>8) || rxbuf[len(rxbuf)-1] != byte(rxcrc&0xFF) {
		return nil, ErrCorruptResponse
	}
	return &response{
		Opcode: opcode(rxbuf[2]),
		Status: int16(rxbuf[3])<<8 | int16(rxbuf[4]),
		Data:   rxbuf[5 : len(rxbuf)-2],
	}, nil
}

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
		crc = ((crc << 4) | (uint16(buf[i]) >> 4)) ^ crcTable[crc>>12]
		crc = ((crc << 4) | (uint16(buf[i]) & 0x0F)) ^ crcTable[crc>>12]
	}

	return crc
}

type Tag struct {
	RFU          []byte
	RSSI         int
	FrequencyMhz float32
	Time         time.Time
	Phase        int
	ProtocolID   TagProtocol
	EmbeddedData []byte
	TagPC        []byte
	TagID        []byte
	TagCRC       []byte
}

func getDataBytes(data []byte) int {
	embeddedDataBits := uint16(data[19])<<8 | uint16(data[20])
	if embeddedDataBits%8 > 0 {
		return int(embeddedDataBits)/8 + 1
	}
	return int(embeddedDataBits) / 8
}

func ParseTag(lastKeepAlive time.Time, data []byte) (*Tag, error) {
	if len(data) < 24 {
		return nil, ErrCorruptResponse
	}
	dataBytes := getDataBytes(data)
	if len(data) < int(24+dataBytes) {
		return nil, ErrCorruptResponse
	}
	rfuLength := int(uint16(data[22+dataBytes])<<8|uint16(data[23+dataBytes])) / 8
	if len(data) < int(24+dataBytes+rfuLength) {
		return nil, ErrCorruptResponse
	}

	t := Tag{
		RFU:          data[:7],
		RSSI:         int(data[7]) - 256,
		FrequencyMhz: float32(uint32(data[9])<<16|uint32(data[10])<<8|uint32(data[11])) / 1000,
		Time:         lastKeepAlive.Add(time.Millisecond * time.Duration(uint32(data[12])<<24|uint32(data[13])<<16|uint32(data[14])<<8|uint32(data[15]))),
		Phase:        int(uint16(data[16])<<8 | uint16(data[17])),
		ProtocolID:   TagProtocol(data[18]),
		EmbeddedData: data[21 : 21+dataBytes],
		TagPC:        data[24+dataBytes : 24+dataBytes+2],
		TagID:        data[24+dataBytes+2 : 24+dataBytes+rfuLength-2],
		TagCRC:       data[24+dataBytes+rfuLength-2 : 24+dataBytes+rfuLength],
	}
	return &t, nil
	//  [0 to 6] 10 00 1B 01 FF 01 01 = RFU 7 bytes
	//  [7] C4 = RSSI
	//  [8] 11 = Antenna ID (4MSB = TX, 4LSB = RX)
	//  [9, 10, 11] 0E 16 40 = Frequency in kHz
	//  [12, 13, 14, 15] 00 00 01 27 = Timestamp in ms since last keep alive msg
	//  [16, 17] 00 00 = phase of signal tag was read at (0 to 180)
	//  [18] 05 = Protocol ID
	//  [19, 20] 00 00 = Number of bits of embedded tag data [M bytes]
	//  [21 to M] (none) = Any embedded data
	//  [21 + M] 0F = RFU reserved future use
	//  [22, 23 + M] 00 80 = EPC Length [N bytes]  (bits in EPC including PC and CRC bits). 128 bits = 16 bytes
	//  [24, 25 + M] 30 00 = Tag EPC Protocol Control (PC) bits
	//  [26 to 37 + M + N] 00 00 00 00 00 00 00 00 00 00 15 45 = EPC ID
	//  [38, 39 + M + N] 45 E9 = EPC CRC

}
