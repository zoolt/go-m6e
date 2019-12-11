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

func (m *M6E) SetBaud(baudRate int) {
}

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

func (m *M6E) SetWritePower(powerSetting uint) {
}

func (m *M6E) GetWritePower() {
	power, err := m.sendMessage(getWriteTxPower, []byte{0x00}, true)
	log.Println(power)
	log.Println(err)
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

	//Remove anything in the incoming buffer
	//TODO this is a bad idea if we are constantly readings tags
	//  while (_nanoSerial->available())
	//    _nanoSerial->read();

	_, err := m.port.Write(txbuf)
	if err != nil {
		return nil, err
	}

	if !waitForResponse {
		return nil, nil
	}

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
	if rxbuf[2] != byte(oc) {
		return nil, ErrWrongOpcodeResponse
	}
	return rxbuf[5 : len(rxbuf)-2], nil
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
