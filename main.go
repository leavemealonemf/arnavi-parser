package main

import (
	. "arnaviparser/structs"
	"context"
	"crypto/rand"
	"encoding/hex"
	"encoding/json"
	"fmt"
	"log"
	"math"
	"net"
	"net/http"
	"os"
	"strconv"
	"strings"
	"time"

	mg "arnaviparser/db/mongo"

	"github.com/gorilla/mux"
	"github.com/gorilla/websocket"
	"github.com/joho/godotenv"
	"go.mongodb.org/mongo-driver/bson"
	"go.mongodb.org/mongo-driver/mongo"
	"go.mongodb.org/mongo-driver/mongo/options"
)

var devices []*Device

// var connections []*Connection
var connections map[int64]*Connection
var deviceStatusBitPos [][]int
var deviceIdsBytesAssotiation map[int]string
var commands map[string]*Command
var receivedCommands []*ReceivedCommand
var addedImeis []int64
var wsConnections []*websocket.Conn

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

func Checksum(val []byte) string {
	checksum := byte(0)
	for _, b := range val {
		checksum += b
	}

	return hex.EncodeToString([]byte{checksum})
}

func GenСmdTokenHex() (string, error) {
	bytes := make([]byte, 4)
	_, err := rand.Read(bytes)
	if err != nil {
		return "", fmt.Errorf("GenСmdTokenHex err: %v\n", err.Error())
	}
	bytes[0] = 0xFF
	return hex.EncodeToString(bytes), nil
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
	if connections[conn.Device.IMEI] != nil {
		delete(connections, conn.Device.IMEI)
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
		if device.SimOne == nil {
			device.SimOne = &SimStatus{}
		}
		device.SimOne.LAC = lac
		device.SimOne.CellID = cellID
	} else {
		if device.SimTwo == nil {
			device.SimTwo = &SimStatus{}
		}
		device.SimTwo.LAC = lac
		device.SimTwo.CellID = cellID
	}
}

func ParseTags8And201(revBytes []byte, device *Device, simNum uint8) {
	if len(revBytes) != 4 {
		return
	}

	signalLevel := revBytes[0]                          // Уровень сигнала (0-31)
	mcc := uint16(revBytes[1])<<8 | uint16(revBytes[2]) // Mobile Country Code (MCC)
	mnc := revBytes[3]                                  // Mobile Network Code (MNC)

	operator := map[byte]string{
		0x01: "МТС",
		0x02: "МегаФон",
		0x07: "СМАРТС",
		0x99: "Билайн",
	}[mnc]

	if simNum == 1 {
		if device.SimOne == nil {
			device.SimOne = &SimStatus{}
		}
		device.SimOne.GSMSigLvl = signalLevel
		device.SimOne.MobileCountyCode = mcc
		device.SimOne.MobileNetCode = mnc
		device.SimOne.OperatorName = operator
	} else {
		if device.SimTwo == nil {
			device.SimTwo = &SimStatus{}
		}
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

func ParseTAG6(hexStringRev string, device *Device) {
	value := hexToDec(hexStringRev)

	ignitionState := (value >> 0) & 0x01
	doorLock1State := (value >> 8) & 0x01
	doorLock2State := (value >> 9) & 0x01
	flashlightState := (value >> 16) & 0x01
	usbPowerState := (value >> 18) & 0x01

	device.TagSix = map[string]int64{
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
		Conn:   conn,
		Device: &device,
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

			var start int64 = 4
			isBrokePackage := false

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
							param := tagParam[2:len(tagParam)]
							p := BytesToHexString(reverseBytes(param))
							ParseTAG6(p, &device)
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

						// case 253, 254, 202, 230:
						// 	if tagIDDec == 253 || tagIDDec == 202 {
						// 		processICCID(reverseBytes(tagParam), &device, 1)
						// 	} else {
						// 		processICCID(reverseBytes(tagParam), &device, 2)
						// 	}
						// 	break
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
					pktType := hexPackageData[start : start+2]
					// pktDatLen := pktBy[offset+2 : 2]
					// pktTs := pktBy[offset+4 : 4]
					start += 2
					start += 4
					start += 8
					errCode := hexPackageData[start : start+2]
					start += 2
					token := "ff" + hexPackageData[start:start+6]
					start += 6
					cs := hexPackageData[start : start+2]
					start += 2

					for _, v := range receivedCommands {
						if v.Status == "completed" || v.Status == "error" {
							continue
						}

						if strings.Compare(v.Token, token) == 0 {
							if strings.Compare(errCode, "00") == 0 {
								v.Status = "completed"
							} else {
								v.Status = "error"
							}

							f := bson.D{{Key: "token", Value: token}}

							upd := bson.D{
								{"$set", bson.D{
									{Key: "status", Value: v.Status},
									{Key: "_ct", Value: time.Now().UnixMicro()},
								}},
							}
							mg.UpdOne(ctx, cmdsColl, f, upd)
						}
					}

					fmt.Println(pktType, errCode, token, cs)
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

			device.ServerTime = time.Now().UnixMicro()
			BindDeviceMainPropertys(&device)
			device.Online = true
			marshal, _ := json.Marshal(device)
			fmt.Println(string(marshal))
			for _, v := range wsConnections {
				v.WriteJSON(device)
			}
			_, e := scooterColl.InsertOne(ctx, device)
			if e != nil {
				fmt.Println("Insert scooter error")
			} else {
				fmt.Println("Insert scooter successfully")
			}
		}
	}
}

func printHexPacketStructData(packet *HEXPacket) {
	fmt.Printf("type of content: %v\ndata len: %v\npacket unixtime: %v\npacket tags data: %v\nchecksum: %v\n\n", packet.TypeOfContent, packet.PacketDataLen, packet.Unixtime, packet.TagsData, packet.Checksum)
}

func HTTPCmdHandlerCustom(w http.ResponseWriter, r *http.Request) {
	if r.Method == "GET" {
		vars := mux.Vars(r)
		imei := vars["imei"]
		cmd := vars["cmd"]
		decImei, _ := strconv.Atoi(imei)

		c := connections[int64(decImei)]

		if c != nil {
			token, _ := GenСmdTokenHex()
			tokenBy := HexToBytes(token)
			cmdBy := HexToBytes(cmd)

			totalBy := make([]byte, 2)
			totalBy = append(totalBy, tokenBy...)
			totalBy = append(totalBy, cmdBy...)

			cs := Checksum(totalBy)

			command := fmt.Sprintf("7B08FF%s%s%s7D", cs, token, cmd)
			sComPackage, _ := hex.DecodeString(command)

			cmdInfo := commands[cmd]

			if cmdInfo == nil {
				fmt.Fprintf(w, "this command does not exist %v", cmd)
				return
			}

			recievedCmd := &ReceivedCommand{
				ServerTime: time.Now().UnixMicro(),
				CMD:        command,
				Token:      token,
				Status:     "pending",
				IMEI:       imei,
				CMDInfo:    commands[cmd],
			}

			receivedCommands = append(receivedCommands, recievedCmd)
			mg.Insert(ctx, cmdsColl, recievedCmd)

			c.Conn.Write(sComPackage)
		}

		fmt.Fprintf(w, "success %v", imei)
	}
}

func HTTPIotDataScooters(w http.ResponseWriter, r *http.Request) {
	w.Header().Add("Content-Type", "application/json")

	if r.Method == "GET" {

		var response []IOTResponse

		for _, v := range addedImeis {
			var dvce *Device
			var r IOTResponse
			mg.FindOneWithOpts(
				ctx, scooterColl,
				bson.D{{Key: "imei", Value: v}},
				options.FindOne().SetSort(bson.D{{Key: "_ts", Value: -1}}),
			).Decode(&dvce)

			if dvce == nil {
				continue
			}

			r.ID = dvce.IMEI
			r.State = dvce

			response = append(response, r)
		}

		if len(response) == 0 {
			fmt.Fprintln(w, "[]")
			return
		}

		j, _ := json.Marshal(response)
		fmt.Fprintln(w, string(j))
	}
}

func HTTPIotFullDataForOneScooter(w http.ResponseWriter, r *http.Request) {
	w.Header().Add("Content-Type", "application/json")

	if r.Method == "GET" {
		vars := mux.Vars(r)
		imei := vars["imei"]

		decImei, _ := strconv.Atoi(imei)
		f := bson.D{{Key: "imei", Value: decImei}}

		curr := mg.FindAllWithOpts(
			ctx, scooterColl,
			f,
			options.Find().SetSort(bson.D{{Key: "_ts", Value: -1}}),
		)

		if curr == nil {
			fmt.Fprintln(w, "[]")
			return
		}

		var res []Device
		curr.All(ctx, &res)

		if len(res) == 0 {
			fmt.Fprintln(w, "[]")
			return
		}

		j, _ := json.Marshal(res)
		fmt.Fprintln(w, string(j))
	}
}

func HTTPIotLatestOneScooterData(w http.ResponseWriter, r *http.Request) {
	w.Header().Add("Content-Type", "application/json")

	if r.Method == "GET" {
		vars := mux.Vars(r)
		imei := vars["imei"]

		decImei, _ := strconv.Atoi(imei)

		var res Device
		var rMap map[string]any

		mg.FindOneWithOpts(
			ctx, scooterColl,
			bson.D{{Key: "imei", Value: decImei}},
			options.FindOne().SetSort(bson.D{{Key: "_ts", Value: -1}}),
		).Decode(&res)

		rMap = map[string]any{
			"id":    res.IMEI,
			"state": res,
		}

		j, _ := json.Marshal(rMap)
		fmt.Fprintln(w, string(j))
	}
}

func HTTPIotLatestOneScooterDataWithCmdsJournal(w http.ResponseWriter, r *http.Request) {
	w.Header().Add("Content-Type", "application/json")

	if r.Method == "GET" {
		vars := mux.Vars(r)
		imei := vars["imei"]

		decImei, _ := strconv.Atoi(imei)

		var res Device
		var cmds []ReceivedCommand

		mg.FindOneWithOpts(
			ctx, scooterColl,
			bson.D{{Key: "imei", Value: decImei}},
			options.FindOne().SetSort(bson.D{{Key: "_ts", Value: -1}}),
		).Decode(&res)

		f := bson.D{{Key: "dvce_imei", Value: imei}}

		cmdsCurr := mg.FindAllWithOpts(ctx, cmdsColl, f, options.Find().SetSort(bson.D{{Key: "_ts", Value: -1}}))
		err := cmdsCurr.All(ctx, &cmds)

		if err != nil {
			fmt.Println("Get commands journal err:", err.Error())
		}

		iotResp := IOTResponseJournal{
			ID:         res.IMEI,
			State:      &res,
			CMDJournal: cmds,
		}

		j, _ := json.Marshal(iotResp)
		fmt.Fprintln(w, string(j))
	}
}

func HTTPIotOneScooterCmdsJournal(w http.ResponseWriter, r *http.Request) {
	w.Header().Add("Content-Type", "application/json")

	if r.Method == "GET" {
		vars := mux.Vars(r)
		imei := vars["imei"]

		var cmds []ReceivedCommand

		f := bson.D{{Key: "dvce_imei", Value: imei}}

		cmdsCurr := mg.FindAllWithOpts(ctx, cmdsColl, f, options.Find().SetSort(bson.D{{Key: "_ts", Value: -1}}))
		err := cmdsCurr.All(ctx, &cmds)

		if err != nil {
			fmt.Println("Get commands journal err:", err.Error())
			fmt.Fprintln(w, "[]")
			return
		}

		j, _ := json.Marshal(cmds)
		fmt.Fprintln(w, string(j))
	}
}

var upgrader = websocket.Upgrader{
	CheckOrigin: func(r *http.Request) bool {
		return true
	},
}

func PullToWS(w http.ResponseWriter, r *http.Request) {
	conn, err := upgrader.Upgrade(w, r, nil)
	if err != nil {
		fmt.Println("Connect to WS err:", err.Error())
		return
	}

	wsConnections = append(wsConnections, conn)

	for {
		_, _, err := conn.ReadMessage()
		if err != nil {
			fmt.Println("[WS] Read message error. Exit", err.Error())
			break
		}

		// m := []byte("hello from iot king")

		// conn.WriteMessage(websocket.TextMessage, m)
		// go WSMessageHandler(m)
	}

	defer conn.Close()
}

func WSMessageHandler(msg []byte) {
	fmt.Println(string(msg))
}

func secureMiddleware(next http.Handler) http.Handler {
	return http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		k, _ := os.LookupEnv("ACCESS_KEY")

		authH := r.Header.Get("Authorization")

		if len(authH) == 0 || strings.Compare(k, authH) != 0 {
			w.WriteHeader(403)
			fmt.Fprintf(w, "Unauthorized")
			return
		}

		next.ServeHTTP(w, r)
	})
}

func bootHTTP() {
	r := mux.NewRouter()
	// r.Use(secureMiddleware)
	r.HandleFunc("/iot/ws", PullToWS)
	r.HandleFunc("/cmds/{imei}/{cmd}", HTTPCmdHandlerCustom)
	r.HandleFunc("/iot/scooters", HTTPIotDataScooters)
	r.HandleFunc("/iot/scooters/{imei}/all", HTTPIotFullDataForOneScooter)
	r.HandleFunc("/iot/scooters/{imei}/latest", HTTPIotLatestOneScooterData)
	r.HandleFunc("/iot/scooters/{imei}/latest/journal", HTTPIotLatestOneScooterDataWithCmdsJournal)
	r.HandleFunc("/iot/scooters/{imei}/cjournal", HTTPIotOneScooterCmdsJournal)
	http.Handle("/", r)
	fmt.Println("Served HTTP on :8080")

	http.ListenAndServe(":8080", nil)
}

func initIOTCommands() {
	commands = map[string]*Command{}
	commands["51330001"] = &Command{
		Val:    "51330001",
		NameEn: "Block Scooter Motor Wheel",
		NameRu: "Блокировка мотор колеса",
	}
	commands["51330000"] = &Command{
		Val:    "51330000",
		NameEn: "Disable Guard Mode",
		NameRu: "Снять с режима охраны",
	}
	commands["51330005"] = &Command{
		Val:    "51330005",
		NameEn: "Enable Guard Mode",
		NameRu: "Установить режим охраны",
	}
	commands["51330006"] = &Command{
		Val:    "51330006",
		NameEn: "Set Service Mode",
		NameRu: "Установить режим сервиса",
	}
	commands["51330300"] = &Command{
		Val:    "51330300",
		NameEn: "Set Drive Mode D",
		NameRu: "Установить стиль езды D",
	}
	commands["51330301"] = &Command{
		Val:    "51330301",
		NameEn: "Set Drive Mode ECO",
		NameRu: "Установить стиль езды ECO",
	}
	commands["51330302"] = &Command{
		Val:    "51330302",
		NameEn: "Set Drive Mode S",
		NameRu: "Установить стиль езды S",
	}
	commands["51080201"] = &Command{
		Val:    "51080201",
		NameEn: "Turn On Flash",
		NameRu: "Включить фонарь",
	}
	commands["51080200"] = &Command{
		Val:    "51080200",
		NameEn: "Turn Off Flash",
		NameRu: "Выключить фонарь",
	}
}

var scooterColl *mongo.Collection
var cmdsColl *mongo.Collection
var ctx = context.TODO()

func init() {
	if err := godotenv.Load(); err != nil {
		log.Fatalln("No .env file found. Exit")
	}
}

func main() {
	mongUsr, _ := os.LookupEnv("MONGO_USR")
	mongPass, _ := os.LookupEnv("MONGO_PASSWORD")
	mongHost, _ := os.LookupEnv("MONGO_HOST")
	mongPort, _ := os.LookupEnv("MONGO_PORT")
	connStr := fmt.Sprintf("mongodb://%s:%s@%s:%s", mongUsr, mongPass, mongHost, mongPort)
	mgClient, err := mg.Connect(ctx, connStr)

	if (err) != nil {
		log.Fatalln(err.Error())
	}

	if mgClient.Ping(ctx, nil) == nil {
		fmt.Println("ping")
	}

	mg.Seed(mgClient, ctx)

	scooterColl = mgClient.Database("iot").Collection("scooters")
	cmdsColl = mgClient.Database("iot").Collection("cmds")

	serve, err := net.Listen("tcp", ":20550")

	devices = make([]*Device, 0)
	connections = map[int64]*Connection{}
	deviceStatusBitPos = [][]int{{27, 26}, {25}, {24, 23}, {22}, {21, 20}, {19, 18}, {17, 16}, {15, 14}, {13, 12}, {11, 10, 9}, {8, 7, 6}, {5}, {4, 3, 2}, {1, 0}}
	deviceIdsBytesAssotiation = map[int]string{
		0: "device_status", 1: "bt", 2: "msd", 3: "guard_zone_ctrl", 4: "mw", 5: "s3_st", 6: "s2_st", 7: "s1_st", 8: "s0_st", 9: "sim2_st", 10: "sim1_st", 11: "sim_t", 12: "gsm_st", 13: "nav_st",
	}
	addedImeis = append(addedImeis, 866011050296805, 866039048453774)

	initIOTCommands()

	if err != nil {
		log.Fatalln("Startup serve error:", err.Error())
	}

	log.Println("Server started:", serve.Addr().Network())

	// view cmds statuses
	go func() {
		for {
			if len(receivedCommands) > 0 {
				fmt.Println("COMMANDS STATUS:")
				for _, v := range receivedCommands {
					fmt.Println("------------")
					fmt.Printf("\ncmd: %v\ntoken: %v\nstatus: %v\n\n", v.CMD, v.Token, v.Status)
					fmt.Println("------------")
				}
			}

			time.Sleep(time.Second * 20)
		}
	}()

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
