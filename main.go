package main

import (
	"encoding/hex"
	"encoding/json"
	"fmt"
	"log"
	"net"
	"net/http"
	"strconv"
	"strings"
	"time"

	"github.com/gorilla/mux"
)

var testDevices []int64
var devices []*Device

// var connections []*Connection
var connections map[int64]*Connection
var deviceStatusBitPos [][]int
var deviceIdsBytesAssotiation map[int]string

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
	MotorRunning   bool   `json:"motor_running"`
	Mode           string `json:"mode"`
	Charging       bool   `json:"charging"`
	ScreenOff      bool   `json:"screen_off"`
	PedestrianMode bool   `json:"pedestrian_mode"`
	Overheat       bool   `json:"overheat"`
	ScooterType    uint8  `json:"scooter_type"`
}

type DeviceVS struct {
	SpeedKMH                   uint8                  `json:"speed_kmh"`
	AverageBatteryCharge       uint8                  `json:"avg_battery_charge"`
	StatementFlags             DeviceVSStatementFlags `json:"statement_flags"`
	MainBatteryCharge          uint8                  `json:"main_battery_charge"`
	AdditionalBatteryCharge    uint8                  `json:"additional_battery_charge"`
	ErrorCode                  uint8                  `json:"err_code"`
	MileagePerTrip             uint32                 `json:"mileage_per_trip"`
	MotorWheelControllerErrors uint16                 `json:"motor_wheel_controller_errors"`
	BMSErrors                  uint8                  `json:"bms_errors"`
}

type Device struct {
	ServerTime int64   `json:"_ts"`
	Timestamp  int64   `json:"time"`
	Online     bool    `json:"online"`
	Charge     uint8   `json:"charge"`       // vs_64 aka self.VirtualSensors.MainBatteryCharge (binded)
	Speed      uint8   `json:"speed"`        // vs_64 aka self.VirtualSensors.SpeedKMH (binded)
	Alt        uint16  `json:"altitude"`     // tag_5.altitude
	Azimut     uint16  `json:"azimut"`       // tag_5.azimut
	Lat        float32 `json:"lat"`          // tag_3
	Lon        float32 `json:"lon"`          // tag_4
	IsSim      bool    `json:"isSim"`        // tag_99 [sim_1st] && [sim_2st] aka self.DeviceStatus["sim_1st"] && self.DeviceStatus["sim_2st"] (binded)
	SimNumber  uint8   `json:"simNumber"`    // tag_99 [sim_t] aka self.DeviceStatus["sim_t"] (binded)
	MoveSensor bool    `json:"mover_sensor"` // tag_99 [mv] aka self.DeviceStatus["mv"] (binded)
	SatGps     uint8   `json:"sat_gps"`      // tag_5
	SatGlonass uint8   `json:"sat_glonass"`  // tag_5
	GPS        uint8   `json:"gps"`          // tag_99 [nav_st] aka self.DeviceStatus["nav_st"] (binded)
	GSM        uint8   `json:"gsm"`          // tag_99 [gsm_st] aka self.DeviceStatus["gsm_st"] (binded)
	LockStatus bool    `json:"lock-status"`  // tag_99 [device_status] aka self.DeviceStatus["device_status"] (binded)
	Charging   bool    `json:"charging"`     // vs_63 [device_status] aka self.VirtualSensors.StatementFlags.Charging (binded)
	// Код сотовой сети
	Mnc            uint32         `json:"mnc"`
	Level          uint32         `json:"level"`
	IMEI           int64          `json:"imei"`
	DeviceStatus   map[string]int `json:"device_status"`
	VirtualSensors DeviceVS       `json:"virtual_sensors"`
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

func BindDeviceMainPropertys(device *Device) {
	device.Charge = device.VirtualSensors.MainBatteryCharge
	device.Speed = device.VirtualSensors.SpeedKMH
	device.MoveSensor = device.DeviceStatus["mw"] == 1
	device.SimNumber = uint8(device.DeviceStatus["sim_t"])
	device.GPS = uint8(device.DeviceStatus["nav_st"])
	device.GSM = uint8(device.DeviceStatus["gsm_st"])
	device.Charging = device.VirtualSensors.StatementFlags.Charging
	device.LockStatus = device.DeviceStatus["device_status"] == 2 || device.DeviceStatus["device_status"] == 3
	if device.DeviceStatus["sim_t"] == 0 {
		device.IsSim = device.DeviceStatus["sim1_st"] == 1
	} else {
		device.IsSim = device.DeviceStatus["sim2_st"] == 1
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

			// VALIDATE IMEI IN DATABASE
			for i := 0; i < len(testDevices); i++ {
				if decIMEI == testDevices[i] {
					fmt.Println("Find imei. Continue...")
					break
				} else {
					fmt.Println("Find imei error. Break...")
					conn.Close()
					break
				}
			}
			// END VALIDATE IMEI IN DATABASE

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

			fmt.Printf("FULL PACKAGE: %v\n\n", hexPackageData)

			fmt.Println("----- PACKETS ------")

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
					fmt.Println("packet checksum:", packetChecksum)

					if strings.ToLower(packetChecksum) != hexPacket.Checksum {
						fmt.Println("Wrong packet checksum. Break...")
						sendServerComFailed("230", conn)
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
							device.DeviceStatus = map[string]int{}

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
								device.DeviceStatus[assotiation] = devicePreResult[i]
							}

							break
						case 6:
							break
						default:
							break
						}

						fmt.Printf("decimal tag_id: %v\nfull hex_tag: %v\ntag_param_without_id: %v\n", tagIDDec, tagFull, tagParam)
						fmt.Println("--------------------")
					}

					printHexPacketStructData(hexPacket)

					if hexPackageData[start:start+2] == "5d" {
						sendServerComSuccessed("239", conn)
						fmt.Println("Packet's parsed successfully")
						break
					}

				} else {
					sendServerComFailed("245", conn)
					break
				}
			}

			timeNow := time.Now()
			device.ServerTime = timeNow.UnixNano()
			BindDeviceMainPropertys(&device)
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

func bootHTTP() {
	r := mux.NewRouter()
	r.HandleFunc("/on/{imei}", HTTPCmdHandlerOn)
	r.HandleFunc("/off/{imei}", HTTPCmdHandlerOn)
	http.Handle("/", r)
	http.ListenAndServe(":8080", nil)
}

func main() {
	serve, err := net.Listen("tcp", ":20550")

	devices = make([]*Device, 0)
	testDevices = make([]int64, 0)
	testDevices = append(testDevices, 866011050296805)
	connections = map[int64]*Connection{}
	deviceStatusBitPos = [][]int{{27, 26}, {25}, {24, 23}, {22}, {21, 20}, {19, 18}, {17, 16}, {15, 14}, {13, 12}, {11, 10, 9}, {8, 7, 6}, {5}, {4, 3, 2}, {1, 0}}
	deviceIdsBytesAssotiation = map[int]string{
		0: "device_status", 1: "bt", 2: "msd", 3: "guard_zone_ctrl", 4: "mw", 5: "s3_st", 6: "s2_st", 7: "s1_st", 8: "s0_st", 9: "sim2_st", 10: "sim1_st", 11: "sim_t", 12: "gsm_st", 13: "nav_st",
	}

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
