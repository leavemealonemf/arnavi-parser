package main

import (
	"context"
	"encoding/hex"
	"encoding/json"
	"fmt"
	"log"
	"math"
	"net"
	"net/http"
	"strconv"
	"strings"
	"time"

	"github.com/gorilla/mux"
	"go.mongodb.org/mongo-driver/mongo"
)

var testDevices []int64
var devices []*Device

// var connections []*Connection
var connections map[int64]*Connection
var deviceStatusBitPos [][]int
var deviceIdsBytesAssotiation map[int]string
var commands map[string]*Command

type Connection struct {
	conn   net.Conn
	device *Device
}

type HEXHeader struct {
	HeaderID    string
	ProtocolVer string
	IMEI        string
	WifiID      string
}

type HEXPackage struct {
	StartSign string
	ParcelNum string
	Packet    *HEXPacket
	EndSign   string
}

type HEXPacket struct {
	TypeOfContent string
	PacketDataLen string
	Unixtime      string
	TagsData      string
	Checksum      string
}

type DeviceVSStatementFlags struct {
	MotorRunning   bool   `json:"motor_running" bson:"motor_running,omitempty"`
	Mode           string `json:"mode" bson:"mode,omitempty"`
	Charging       bool   `json:"charging" bson:"charging,omitempty"`
	ScreenOff      bool   `json:"screen_off" bson:"screen_off,omitempty"`
	PedestrianMode bool   `json:"pedestrian_mode" bson:"pedestrian_mode,omitempty"`
	Overheat       bool   `json:"overheat" bson:"overheat,omitempty"`
	ScooterType    uint8  `json:"scooter_type" bson:"scooter_type,omitempty"`
}

type DeviceVS struct {
	SpeedKMH                   uint8                  `json:"speed_kmh" bson:"speed_kmh,omitempty"`
	AverageBatteryCharge       uint8                  `json:"avg_battery_charge" bson:"avg_battery_charge,omitempty"`
	MainBatteryCharge          uint8                  `json:"main_battery_charge" bson:"main_battery_charge.omitempty"`
	AdditionalBatteryCharge    uint8                  `json:"additional_battery_charge" bson:"additional_battery_charge,omitempty"`
	ErrorCode                  uint8                  `json:"err_code" bson:"err_code,omitempty"`
	MileagePerTrip             uint32                 `json:"mileage_per_trip" bson:"mileage_per_trip,omitempty"`
	MotorWheelControllerErrors uint16                 `json:"motor_wheel_controller_errors" bson:"motor_wheel_controller_errors,omitempty"`
	BMSErrors                  uint8                  `json:"bms_errors" bson:"bms_errors,omitempty"`
	StatementFlags             DeviceVSStatementFlags `json:"statement_flags" bson:"statement_flags,omitempty"`
}

type TAGFive struct {
	SpeedKnots      float64 `json:"speed" bson:"speed,omitempty"`
	Alt             int     `json:"altitude" bson:"altitude,omitempty"`
	Azimut          int     `json:"azimut" bson:"azimut,omitempty"`
	SatGps          uint8   `json:"sat_gps" bson:"sat_gps,omitempty"`
	SatGlonass      uint8   `json:"sat_glonass" bson:"sat_glonass,omitempty"`
	TotalSatellites byte    `json:"total_sat" bson:"total_sat,omitempty"`
}

type TAGOne struct {
	ExternalVolt uint16 `json:"ext_volt" bson:"ext_volt,omitempty"`
	InternalVilt uint16 `json:"int_volt" bson:"int_volt,omitempty"`
}

type SimStatus struct {
	LAC              uint16 `json:"lac" bson:"lac,omitempty"`
	CellID           uint16 `json:"cell_id" bson:"cell_id,omitempty"`
	GSMSigLvl        uint8  `json:"sig_lvl_gsm" bson:"sig_lvl_gsm,omitempty"`
	MobileNetCode    uint8  `json:"mnc" bson:"mnc,omitempty"`
	MobileCountyCode uint16 `json:"mcc" bson:"mcc,omitempty"`
	OperatorName     string `json:"operator_name" bson:"operator_name,omitempty"`
}

type Device struct {
	ServerTime      int64             `json:"_ts" bson:"_ts,omitempty"`
	Timestamp       int64             `json:"time" bson:"time,omitempty"`
	Online          bool              `json:"online" bson:"online,omitempty"`
	IMEI            int64             `json:"imei" bson:"imei,omitempty"`
	Speed           uint8             `json:"speed_kmh" bson:"speed_kmh,omitempty"`       // vs_64 aka self.VirtualSensors.SpeedKMH (binded)
	Charge          uint8             `json:"charge" bson:"charge,omitempty"`             // vs_64 aka self.VirtualSensors.MainBatteryCharge (binded)
	SpeedKnots      float64           `json:"speed" bson:"speed,omitempty"`               // tag_5.speed (binded)
	Alt             int               `json:"altitude" bson:"altitude,omitempty"`         // tag_5.altitude (binded)
	Azimut          int               `json:"azimut" bson:"azimut,omitempty"`             // tag_5.azimut (binded)
	Lat             float32           `json:"lat" bson:"lat,omitempty"`                   // tag_3 (binded)
	Lon             float32           `json:"lon" bson:"lon,omitempty"`                   // tag_4 (binded)
	IsSim           bool              `json:"isSim" bson:"isSim,omitempty"`               // tag_99 [sim_1st] && [sim_2st] aka self.DeviceStatus["sim_1st"] && self.DeviceStatus["sim_2st"] (binded)
	SimNumber       uint8             `json:"simNumber" bson:"simNumber,omitempty"`       // tag_99 [sim_t] aka self.DeviceStatus["sim_t"] (binded)
	MoveSensor      bool              `json:"mover_sensor" bson:"mover_sensor,omitempty"` // tag_99 [mv] aka self.DeviceStatus["mv"] (binded)
	SatGps          uint8             `json:"sat_gps" bson:"sat_gps,omitempty"`           // tag_5 (binded)
	SatGlonass      uint8             `json:"sat_glonass" bson:"sat_glonass,omitempty"`   // tag_5 (binded)
	TotalSatellites byte              `json:"total_sat" bson:"total_sat,omitempty"`       // tag_5 (binded)
	GPS             uint8             `json:"gps" bson:"gps,omitempty"`                   // tag_99 [nav_st] aka self.DeviceStatus["nav_st"] (binded)
	GSM             uint8             `json:"gsm" bson:"gsm,omitempty"`                   // tag_99 [gsm_st] aka self.DeviceStatus["gsm_st"] (binded)
	LockStatus      bool              `json:"lock-status" bson:"lock-status,omitempty"`   // tag_99 [device_status] aka self.DeviceStatus["device_status"] (binded)
	Charging        bool              `json:"charging" bson:"charging,omitempty"`         // vs_63 [device_status] aka self.VirtualSensors.StatementFlags.Charging (binded)
	Mnc             uint32            `json:"mnc" bson:"mnc,omitempty"`                   // tag_7 cellID
	DeviceStatus2   map[string]int    `json:"tag_99" bson:"tag_99,omitempty"`
	DeviceStatus1   map[string]uint32 `json:"tag_9" bson:"tag_9,omitempty"`
	TagSix          map[string]uint32 `json:"tag_6" bson:"tag_6,omitempty"`
	TagFive         TAGFive           `json:"tag_5" bson:"tag_5,omitempty"`
	TAGOne          TAGOne            `json:"tag_1" bson:"tag_1,omitempty"`
	SimOne          SimStatus         `json:"sim_1" bson:"sim_1,omitempty"`
	SimTwo          SimStatus         `json:"sim_2" bson:"sim_2,omitempty"`
	VirtualSensors  DeviceVS          `json:"vs" bson:"vs,omitempty"`
	ICCIDParts      map[uint8][]byte  `json"-"`
}

type Command struct {
	Val string
}

func BytesToHexString(bytes []byte) string {
	encoded := hex.EncodeToString(bytes)
	return encoded
}

func HexToBytes(hexString string) []byte {
	bytes, _ := hex.DecodeString(hexString)
	return bytes
}

func parseIMEI(hexIMEI string) int64 {
	bytes, _ := hex.DecodeString(hexIMEI)

	for i, j := 0, len(bytes)-1; i < j; i, j = i+1, j-1 {
		bytes[i], bytes[j] = bytes[j], bytes[i]
	}

	reversedIMEI := BytesToHexString(bytes)

	decIMEI, _ := strconv.ParseInt(reversedIMEI, 16, 64)
	return decIMEI
}

func reverseBytes(hexString string) []byte {
	bytes, _ := hex.DecodeString(hexString)

	for i, j := 0, len(bytes)-1; i < j; i, j = i+1, j-1 {
		bytes[i], bytes[j] = bytes[j], bytes[i]
	}

	return bytes
}

func hexToDec(hexString string) int64 {
	dec, _ := strconv.ParseInt(hexString, 16, 64)
	return dec
}

func PacketHexChecksum(hexPacket *HEXPacket) string {
	var packetData string = hexPacket.Unixtime + hexPacket.TagsData
	revBytes := reverseBytes(packetData)

	checksum := byte(0)
	for _, b := range revBytes {
		checksum += b
	}

	return fmt.Sprintf("%02x", checksum)
}

func sendServerComSuccessed(codeLine string, conn net.Conn) {
	fmt.Printf("[LINE %v] Get package. Sending SERVER_COM success...\n", codeLine)
	sComPackage, _ := hex.DecodeString("7B00017D")
	// sComPackage, _ := hex.DecodeString("7B02010201017D")
	// sComPackage, _ := hex.DecodeString("7B02010201017D")
	// sComPackage, _ := hex.DecodeString("7B060102DDCCBBAA01017D")

	conn.Write(sComPackage)
}

func sendServerComFailed(codeLine string, conn net.Conn) {
	fmt.Printf("[LINE %v] Get package. Sending SERVER_COM failed...\n", codeLine)
	sComPackage, _ := hex.DecodeString("7B00FE7D")
	conn.Write(sComPackage)
}

func sendTestCMD(conn net.Conn) {
	fmt.Printf("Send COMMAND...\n")
	// sComPackage, _ := hex.DecodeString("7B03FF333300007D")
	// sComPackage, _ := hex.DecodeString("7B03FF343300017D")
	// sComPackage, _ := hex.DecodeString("7B08FF57FF314e55513300007D")
	// sComPackage, _ := hex.DecodeString("7B08FF58FF314e55513300017D")
	sComPackage, _ := hex.DecodeString("7B08FF58FF314e55513300017D")
	// 7B08FF57FF314e55513300007D
	// 7B03FF343300017D
	conn.Write(sComPackage)
}

func DecodeVSStatementFlags(hexStr string, device *Device) {
	data, err := hex.DecodeString(hexStr)
	if err != nil || len(data) < 2 {
		fmt.Printf("[DecodeVSStatementFlags] Invalid hex string: %s", hexStr)
		return
	}

	byte0 := data[0]
	byte1 := data[1]

	motorRunning := byte0&0x01 != 0
	mode := (byte0 >> 1) & 0x03
	charging := byte0&0x08 != 0
	screenOff := byte0&0x10 != 0
	pedestrianMode := byte0&0x20 != 0
	overheat := byte0&0x80 != 0

	modes := map[byte]string{
		0b00: "D",
		0b01: "ECO",
		0b10: "S",
	}

	device.VirtualSensors.StatementFlags.MotorRunning = motorRunning
	device.VirtualSensors.StatementFlags.Mode = modes[mode]
	device.VirtualSensors.StatementFlags.Charging = charging
	device.VirtualSensors.StatementFlags.ScreenOff = screenOff
	device.VirtualSensors.StatementFlags.PedestrianMode = pedestrianMode
	device.VirtualSensors.StatementFlags.Overheat = overheat
	device.VirtualSensors.StatementFlags.ScooterType = byte1
}

func AbortTCPDeviceConn(conn *Connection) {
	if connections[conn.device.IMEI] != nil {
		delete(connections, conn.device.IMEI)
	}
}

func BindDeviceMainPropertys(device *Device) {
	device.Charge = device.VirtualSensors.MainBatteryCharge
	device.Speed = device.VirtualSensors.SpeedKMH
	device.MoveSensor = device.DeviceStatus2["mw"] == 1
	device.SimNumber = uint8(device.DeviceStatus2["sim_t"])
	device.GPS = uint8(device.DeviceStatus2["nav_st"])
	device.GSM = uint8(device.DeviceStatus2["gsm_st"])
	device.Charging = device.VirtualSensors.StatementFlags.Charging
	device.LockStatus = device.DeviceStatus2["device_status"] == 2 || device.DeviceStatus2["device_status"] == 3
	if device.DeviceStatus2["sim_t"] == 0 {
		device.IsSim = device.DeviceStatus2["sim1_st"] == 1
	} else {
		device.IsSim = device.DeviceStatus2["sim2_st"] == 1
	}
}

func ParseTAG1Data(revBytes []byte, device *Device) {
	if len(revBytes) != 4 {
		return
	}
	externalVoltage := uint16(revBytes[0])<<8 | uint16(revBytes[1])
	internalVoltage := uint16(revBytes[2])<<8 | uint16(revBytes[3])
	device.TAGOne.ExternalVolt = externalVoltage
	device.TAGOne.InternalVilt = internalVoltage
}

func ParseTags7And200(revBytes []byte, device *Device, simNum uint8) {
	if len(revBytes) != 4 {
		return
	}

	lac := uint16(revBytes[0])<<8 | uint16(revBytes[1])
	cellID := uint16(revBytes[2])<<8 | uint16(revBytes[3])

	if simNum == 1 {
		device.SimOne.LAC = lac
		device.SimOne.CellID = cellID
	} else {
		device.SimTwo.LAC = lac
		device.SimTwo.CellID = cellID
	}
}

func ParseTags8And201(revBytes []byte, device *Device, simNum uint8) {
	if len(revBytes) != 4 {
		return
	}

	signalLevel := revBytes[0] & 0x1F

	mcc := uint16(revBytes[1])<<8 | uint16(revBytes[2])

	mnc := revBytes[3]

	operator := map[byte]string{
		0x01: "МТС",
		0x02: "МегаФон",
		0x07: "СМАРТС",
		0x99: "Билайн",
	}[mnc]

	if simNum == 1 {
		device.SimOne.GSMSigLvl = signalLevel
		device.SimOne.MobileCountyCode = mcc
		device.SimOne.MobileNetCode = mnc
		device.SimOne.OperatorName = operator
	} else {
		device.SimTwo.GSMSigLvl = signalLevel
		device.SimTwo.MobileCountyCode = mcc
		device.SimTwo.MobileNetCode = mnc
		device.SimTwo.OperatorName = operator
	}
}

func processICCID(revBytes []byte, device *Device, partNum uint8) {
	device.ICCIDParts[partNum] = revBytes
}

func ParseTAG5Data(hexValue string, device *Device) {
	data := reverseBytes(hexValue)

	speedKnots := float64(data[0]) * 1.852
	gpsSatellites := data[1] & 0x0F
	glonassSatellites := (data[1] >> 4) & 0x0F
	totalSatellites := gpsSatellites + glonassSatellites
	altitudeMeters := int(data[2]) * 10
	rate := int(data[3]) * 2

	device.TagFive.SpeedKnots = speedKnots
	device.TagFive.SatGps = gpsSatellites
	device.TagFive.SatGlonass = glonassSatellites
	device.TagFive.Alt = altitudeMeters
	device.TagFive.Azimut = rate
	device.TagFive.TotalSatellites = totalSatellites

	device.SpeedKnots = speedKnots
	device.SatGps = gpsSatellites
	device.SatGlonass = glonassSatellites
	device.Alt = altitudeMeters
	device.Azimut = rate
	device.TotalSatellites = totalSatellites
}

func ParseTAG6(hexString string, device *Device) {
	value := uint32(hexToDec(hexString))

	if (value>>24)&0xFF != 0x01 {
		fmt.Println("Неверный режим входа TAG6. Пропускаем обработку.")
		return
	}

	ignitionState := (value >> 0) & 0x01    // бит 0 - зажигание
	doorLock1State := (value >> 8) & 0x01   // бит 8 - замок 1
	doorLock2State := (value >> 9) & 0x01   // бит 9 - замок 2
	flashlightState := (value >> 16) & 0x01 // бит 16 - фонарик
	usbPowerState := (value >> 18) & 0x01   // бит 18 - питание USB порта

	device.TagSix = map[string]uint32{
		"ignition_st":      ignitionState,
		"door_one_lock_st": doorLock1State,
		"door_two_lock_st": doorLock2State,
		"flash_light_st":   flashlightState,
		"usb_pwr_st":       usbPowerState,
	}
}

func ParseTAG9(hexStr string, device *Device) {
	data, err := hex.DecodeString(hexStr)
	if err != nil || len(data) != 4 {
		return
	}

	value := uint32(data[0])<<24 | uint32(data[1])<<16 | uint32(data[2])<<8 | uint32(data[3])
	voltage := (value >> 24) & 0xFF

	device.DeviceStatus1 = map[string]uint32{
		"in0":     (value >> 0) & 1,
		"in1":     (value >> 1) & 1,
		"in2":     (value >> 2) & 1,
		"in3":     (value >> 3) & 1,
		"in4":     (value >> 4) & 1,
		"in5":     (value >> 5) & 1,
		"in6":     (value >> 6) & 1,
		"in7":     (value >> 7) & 1,
		"out1":    (value >> 8) & 1,
		"out2":    (value >> 9) & 1,
		"out3":    (value >> 10) & 1,
		"gsm_st":  (value >> 12) & 3,
		"nav_st":  (value >> 14) & 3,
		"mw":      (value >> 16) & 1,
		"sim_t":   (value >> 17) & 1,
		"sim_in":  (value >> 18) & 1,
		"st0":     (value >> 19) & 1,
		"st1":     (value >> 20) & 1,
		"st2":     (value >> 21) & 1,
		"pwr_in":  (value >> 22) & 3,
		"pwr_ext": voltage,
	}

}

func handleServe(conn net.Conn) {
	defer conn.Close()

	isFirstConn := true
	// isCmdSended := false

	buff := make([]byte, 5000)

	var device Device

	connection := &Connection{
		conn:   conn,
		device: &device,
	}

	for {
		_, err := conn.Read(buff)
		if err != nil {
			fmt.Println("Received data err:", err.Error())
			AbortTCPDeviceConn(connection)
			break
		}

		hexPackageData := BytesToHexString(buff)
		fmt.Println("Received msg:", hexPackageData)

		if isFirstConn {
			// check data is header

			if strings.ToLower(hexPackageData[0:2]) != "ff" {
				fmt.Println("Not a header. Break")
				break
			}

			header := &HEXHeader{
				HeaderID:    hexPackageData[0:2],
				ProtocolVer: hexPackageData[2:4],
				IMEI:        hexPackageData[4:20],
			}

			// headerHex := HexToBytes(hexPackageData)
			// var headerBytes []byte

			if strings.ToLower(header.ProtocolVer) == "22" {
				// header 1 case
				// 10 bytes header len
				// headerBytes = headerHex[0:10]

			} else if strings.ToLower(header.ProtocolVer) == "23" || strings.ToLower(header.ProtocolVer) == "25" {
				// header 2 case
				// 10 bytes header len
				// headerBytes = headerHex[0:10]

			} else if strings.ToLower(header.ProtocolVer) == "24" {
				// header 3 case
				// 18 bytes header len
				// headerBytes = headerHex[0:18]
			}

			decIMEI := parseIMEI(header.IMEI)
			// isFindImei := false

			// // VALIDATE IMEI IN DATABASE
			// for i := 0; i < len(testDevices); i++ {
			// 	if decIMEI == testDevices[i] {
			// 		fmt.Println("Find imei. Continue...")
			// 		isFindImei = true
			// 		break
			// 	} else {
			// 		fmt.Println("Find imei error. Break...")
			// 		break
			// 	}
			// }

			// if !isFindImei {
			// 	break
			// }

			// // END VALIDATE IMEI IN DATABASE

			device.IMEI = decIMEI

			devices = append(devices, &device)
			connections[decIMEI] = connection

			// send SERVER_COM
			// 7B0400CA5E9F6F5E7D
			// 67A8C24B
			// data, err := hex.DecodeString("7B0400CA5E9F6F5E7D")
			data, _ := hex.DecodeString("7B04001C67A8C24B7D")
			conn.Write(data)
			fmt.Println("sending server com...")
			isFirstConn = false
		} else {
			if strings.ToLower(hexPackageData[0:2]) != "5b" {
				fmt.Println("Wrong package sign. Break...")
				break
			}

			// hexPackage := &HEXPackage{
			// 	StartSign: hexPackageData[0:2],
			// 	ParcelNum: hexPackageData[2:4],
			// }

			// hexPacket := &HEXPacket{
			// 	TypeOfContent: hexPackageData[4:6],
			// 	PacketDataLen: hexPackageData[6:8],
			// 	Unixtime:      hexPackageData[8:16],
			// }

			// if !isCmdSended {
			// 	sendTestCMD(conn)
			// 	isCmdSended = true
			// 	continue
			// }

			var start int64 = 4
			isBrokePackage := false

			// fmt.Printf("FULL PACKAGE: %v\n\n", hexPackageData)
			// fmt.Println("----- PACKETS ------")

			for {
				hexPacket := &HEXPacket{
					TypeOfContent: hexPackageData[start : start+2],
				}

				if strings.ToLower(hexPacket.TypeOfContent) == "01" {
					start += 2 // skip type
					hexPacket.PacketDataLen = hexPackageData[start : start+2]
					dataLenBytes := (hexToDec(hexPackageData[start:start+2]) * 2)
					start += 4 // skip len

					hexPacket.Unixtime = hexPackageData[start : start+8]
					timestamp := hexToDec(BytesToHexString(reverseBytes(hexPacket.Unixtime)))
					device.Timestamp = timestamp

					start += 8 // skip ts
					hexPacket.TagsData = hexPackageData[start : start+dataLenBytes]
					hexPacket.Checksum = hexPackageData[start+dataLenBytes : start+dataLenBytes+2]

					packetChecksum := PacketHexChecksum(hexPacket)
					// fmt.Println("packet checksum:", packetChecksum)

					if strings.ToLower(packetChecksum) != hexPacket.Checksum {
						fmt.Println("Wrong packet checksum. Break...")
						isBrokePackage = true
						break
					}

					start += dataLenBytes + 2

					for i := 0; i < len(hexPacket.TagsData); i = i + 10 {
						if i+10 > len(hexPacket.TagsData) {
							fmt.Println("out of range tags parse")
							break
						}

						tagIDDec := hexToDec(string(hexPacket.TagsData[i : i+2]))
						tagFull := hexPacket.TagsData[i : i+10]
						tagParam := hexPacket.TagsData[i+2 : i+10]

						switch tagIDDec {
						// vs sensors
						case 190:
							internalTagIDDec := hexToDec(string(hexPacket.TagsData[i+2 : i+4]))
							internalTagParamRv := BytesToHexString(reverseBytes(hexPacket.TagsData[i+4 : i+10]))
							switch internalTagIDDec {
							case 61:
								speedKmh := hexToDec(internalTagParamRv)
								device.VirtualSensors.SpeedKMH = uint8(speedKmh)
								break
							case 62:
								avgBtCharge := hexToDec(internalTagParamRv)
								device.VirtualSensors.AverageBatteryCharge = uint8(avgBtCharge)
								break
							case 63:
								internalParam := hexPacket.TagsData[i+4 : i+10]
								DecodeVSStatementFlags(internalParam, &device)
								break
							case 66:
								mainBtCharge := hexToDec(internalTagParamRv)
								device.VirtualSensors.MainBatteryCharge = uint8(mainBtCharge)
							case 67:
								additionalBtCharge := hexToDec(internalTagParamRv)
								device.VirtualSensors.AdditionalBatteryCharge = uint8(additionalBtCharge)
							case 68:
								errCode := hexToDec(internalTagParamRv)
								device.VirtualSensors.ErrorCode = uint8(errCode)
							case 69:
								mileagePerTrip := hexToDec(internalTagParamRv)
								device.VirtualSensors.MileagePerTrip = uint32(mileagePerTrip)
							case 70:
								motorWheelControllerErrors := hexToDec(internalTagParamRv)
								device.VirtualSensors.MotorWheelControllerErrors = uint16(motorWheelControllerErrors)
							case 71:
								bmsError := hexToDec(internalTagParamRv)
								device.VirtualSensors.BMSErrors = uint8(bmsError)
							default:
								break
							}
							break
						// device status
						case 99:
							tagParamRv := BytesToHexString(reverseBytes(tagParam))
							num, _ := strconv.ParseInt(tagParamRv, 16, 64)

							devicePreResult := map[int]int{}
							device.DeviceStatus2 = map[string]int{}

							for i := 0; i < len(deviceStatusBitPos); i++ {
								var result int64
								for j := 0; j < len(deviceStatusBitPos[i]); j++ {
									bit := (num >> deviceStatusBitPos[i][j]) & 1
									result |= bit << (len(deviceStatusBitPos[i]) - j - 1)
								}
								devicePreResult[i] = int(result)
							}

							for i := 0; i < len(devicePreResult); i++ {
								assotiation := deviceIdsBytesAssotiation[i]
								device.DeviceStatus2[assotiation] = devicePreResult[i]
							}

							break

						case 9:
							tagParamRv := BytesToHexString(reverseBytes(tagParam))
							ParseTAG9(tagParamRv, &device)
							break
						case 6:
							ParseTAG6(tagParam, &device)
							break
						case 3, 4:
							rvParamNum := hexToDec(BytesToHexString(reverseBytes(tagParam)))
							v := math.Float32frombits(uint32(rvParamNum))
							if tagIDDec == 3 {
								device.Lat = v
							} else {
								device.Lon = v
							}
							break
						case 5:
							ParseTAG5Data(tagParam, &device)
							break
						case 1:
							ParseTAG1Data(reverseBytes(tagParam), &device)
							break
						case 7, 200:
							if tagIDDec == 7 {
								ParseTags7And200(reverseBytes(tagParam), &device, 1)
							} else {
								ParseTags7And200(reverseBytes(tagParam), &device, 2)
							}
							break
						case 8, 201:
							if tagIDDec == 8 {
								ParseTags8And201(reverseBytes(tagParam), &device, 1)
							} else {
								ParseTags8And201(reverseBytes(tagParam), &device, 2)
							}
							break

						case 253, 254, 202, 230:
							if tagIDDec == 253 || tagIDDec == 202 {
								processICCID(reverseBytes(tagParam), &device, 1)
							} else {
								processICCID(reverseBytes(tagParam), &device, 2)
							}
							break
						default:
							break
						}

						fmt.Printf("decimal tag_id: %v\nfull hex_tag: %v\ntag_param_without_id: %v\n", tagIDDec, tagFull, tagParam)
						fmt.Println("--------------------")
					}

					printHexPacketStructData(hexPacket)

					if hexPackageData[start:start+2] == "5d" {
						// sendServerComSuccessed("506", conn)
						fmt.Println("Packet's parsed successfully")
						break
					}

				} else if strings.ToLower(hexPacket.TypeOfContent) == "09" {
					// sendServerComSuccessed("512", conn)
					break
				} else if strings.ToLower(hexPacket.TypeOfContent) == "08" {
					break
				} else {
					isBrokePackage = true
					// sendServerComFailed("515", conn)
				}
			}

			fmt.Println("is broke", isBrokePackage)
			sendServerComSuccessed("527", conn)
			// if isBrokePackage {
			// 	sendServerComSuccessed("527", conn)
			// } else {
			// 	sendServerComFailed("529", conn)
			// 	break
			// }

			timeNow := time.Now()
			device.ServerTime = timeNow.UnixNano()
			BindDeviceMainPropertys(&device)
			device.Online = true
			marshal, _ := json.Marshal(device)
			fmt.Println(string(marshal))
		}
	}
}

func printHexPacketStructData(packet *HEXPacket) {
	fmt.Printf("type of content: %v\ndata len: %v\npacket unixtime: %v\npacket tags data: %v\nchecksum: %v\n\n", packet.TypeOfContent, packet.PacketDataLen, packet.Unixtime, packet.TagsData, packet.Checksum)
}

func HTTPCmdHandlerOn(w http.ResponseWriter, r *http.Request) {
	if r.Method == "GET" {
		vars := mux.Vars(r)
		imei := vars["imei"]
		decImei, _ := strconv.Atoi(imei)

		c := connections[int64(decImei)]

		if c != nil {
			sComPackage, _ := hex.DecodeString("7B08FF57FF314e55513300007D")
			c.conn.Write(sComPackage)
		}

		fmt.Fprintf(w, "success %v", imei)
	}
}

func HTTPCmdHandlerOff(w http.ResponseWriter, r *http.Request) {
	if r.Method == "GET" {
		vars := mux.Vars(r)
		imei := vars["imei"]
		decImei, _ := strconv.Atoi(imei)

		c := connections[int64(decImei)]

		if c != nil {
			sComPackage, _ := hex.DecodeString("7B08FF58FF314e55513300017D")
			c.conn.Write(sComPackage)
		}

		fmt.Fprintf(w, "success %v", imei)
	}
}

func HTTPCmdHandlerCustom(w http.ResponseWriter, r *http.Request) {
	if r.Method == "GET" {
		vars := mux.Vars(r)
		imei := vars["imei"]
		cmd := vars["cmd"]
		fmt.Println(cmd)
		fmt.Println([]byte(cmd))
		decImei, _ := strconv.Atoi(imei)

		c := connections[int64(decImei)]

		if c != nil {
			sComPackage, _ := hex.DecodeString(cmd)
			c.conn.Write(sComPackage)
		}

		fmt.Fprintf(w, "success %v", imei)
	}
}

func bootHTTP() {
	r := mux.NewRouter()
	r.HandleFunc("/on/{imei}", HTTPCmdHandlerOn)
	r.HandleFunc("/off/{imei}", HTTPCmdHandlerOff)
	r.HandleFunc("/cmds/{imei}/{cmd}", HTTPCmdHandlerCustom)
	http.Handle("/", r)
	http.ListenAndServe(":8080", nil)
}

func initIOTCommands() {
	commands := map[string]*Command{}
	commands["set_block_motor_wheel"] = &Command{
		Val: "7B08FF58FF314E55513300017D",
	}
	commands["unset_block_motor_wheel_and_guard_mode"] = &Command{
		Val: "7B08FF57FF314E55513300007D",
	}
	commands["set_guard_mode"] = &Command{
		Val: "7B08FF5CFF314E55513300057D",
	}
	commands["set_service_mode"] = &Command{
		Val: "7B08FF5DFF314E55513300067D",
	}
	commands["set_d_drive_mode"] = &Command{
		Val: "7B08FF5AFF314E55513303007D",
	}
	commands["set_eco_drive_mode"] = &Command{
		Val: "7B08FF5BFF314E55513303017D",
	}
	commands["set_s_drive_mode"] = &Command{
		Val: "7B08FF5CFF314E55513303027D",
	}
}

var scooterColl *mongo.Collection
var ctx = context.TODO()

func main() {

	// mgClient, err := mg.Connect(ctx, "mongodb://localhost:27017")

	// if (err) != nil {
	// 	log.Fatalln(err.Error())
	// }

	// scooterColl = mgClient.Database("iot").Collection("scooters")

	serve, err := net.Listen("tcp", ":20550")

	devices = make([]*Device, 0)
	testDevices = make([]int64, 0)
	testDevices = append(testDevices, 866011050296805)
	connections = map[int64]*Connection{}
	deviceStatusBitPos = [][]int{{27, 26}, {25}, {24, 23}, {22}, {21, 20}, {19, 18}, {17, 16}, {15, 14}, {13, 12}, {11, 10, 9}, {8, 7, 6}, {5}, {4, 3, 2}, {1, 0}}
	deviceIdsBytesAssotiation = map[int]string{
		0: "device_status", 1: "bt", 2: "msd", 3: "guard_zone_ctrl", 4: "mw", 5: "s3_st", 6: "s2_st", 7: "s1_st", 8: "s0_st", 9: "sim2_st", 10: "sim1_st", 11: "sim_t", 12: "gsm_st", 13: "nav_st",
	}

	initIOTCommands()

	if err != nil {
		log.Fatalln("Startup serve error:", err.Error())
	}

	log.Println("Server started:", serve.Addr().Network())

	go bootHTTP()

	for {
		conn, err := serve.Accept()

		if err != nil {
			log.Fatalln("accept connection error:", err.Error())
		}

		fmt.Printf("Received new connection:\n%v\n%v\n\r", conn.RemoteAddr().String(), conn.LocalAddr().String())

		go handleServe(conn)
	}
}
