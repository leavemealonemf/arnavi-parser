package main

import (
	"encoding/hex"
	"fmt"
	"log"
	"net"
	"strconv"
	"strings"
)

var testDevices []int64
var devices []*Device

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

type Device struct {
	ServerTime uint64  `json:'_ts'`
	Timestamp  uint64  `json:'time'`
	Online     bool    `json:'online'`
	Charge     uint8   `json:'charge'`
	Alt        uint16  `json:'altitude'`
	Azimut     uint16  `json:'azimut'`
	Lat        float32 `json:'lat'`
	Lon        float32 `json:'lon'`
	SatGps     uint8   `json:'sat_gps'`
	SatGlonass uint8   `json:'sat_glonass'`
	// Код сотовой сети
	Mnc   uint32 `json:'mnc'`
	Level uint32 `json:'level'`
	IMEI  int64  `json:'imei'`
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
	sComPackage, _ := hex.DecodeString("7B03FF333300007D")
	conn.Write(sComPackage)
}

func handleServe(conn net.Conn) {
	defer conn.Close()

	isFirstConn := true
	isCmdSended := false

	buff := make([]byte, 5000)

	for {
		_, err := conn.Read(buff)
		if err != nil {
			fmt.Println("Received data err:", err.Error())
			break
		}

		var device Device
		hexPackageData := BytesToHexString(buff)
		// fmt.Println("Received msg:", hexPackageData)

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

			// send SERVER_COM
			// 7B0400CA5E9F6F5E7D
			// 67A8C24B
			// data, err := hex.DecodeString("7B0400CA5E9F6F5E7D")
			data, err := hex.DecodeString("7B04001C67A8C24B7D")

			if err != nil {
				panic(err)
			}

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

			if !isCmdSended {
				sendTestCMD(conn)
				isCmdSended = true
			}

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
					start += 8 // skip ts
					hexPacket.TagsData = hexPackageData[start : start+dataLenBytes]
					// checksum := hexPackageData[start+dataLenBytes : start+dataLenBytes+2]
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

						// switch tagIDDec {
						// case 190:
						// 	break
						// case 99:
						// 	break
						// case 6:
						// 	break
						// default:
						// 	break
						// }

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

			// fmt.Println("[LINE 241] Get package. Sending SERVER_COM...")
			// sComPackage, _ := hex.DecodeString("7B00017D")
			// conn.Write(sComPackage)
		}
	}
}

func printHexPacketStructData(packet *HEXPacket) {
	fmt.Printf("type of content: %v\ndata len: %v\npacket unixtime: %v\npacket tags data: %v\nchecksum: %v\n\n", packet.TypeOfContent, packet.PacketDataLen, packet.Unixtime, packet.TagsData, packet.Checksum)
}

func main() {
	serve, err := net.Listen("tcp", ":20550")

	devices = make([]*Device, 0)
	testDevices = make([]int64, 0)
	testDevices = append(testDevices, 866011050296805)

	if err != nil {
		log.Fatalln("Startup serve error:", err.Error())
	}

	log.Println("Server started:", serve.Addr().Network())

	for {
		conn, err := serve.Accept()

		if err != nil {
			log.Fatalln("accept connection error:", err.Error())
		}

		fmt.Printf("Received new connection:\n%v\n%v\n\r", conn.RemoteAddr().String(), conn.LocalAddr().String())

		go handleServe(conn)
	}
}

// header
// ff23e54506e9a11303000000000000000000000000000000000000000000000

// PACKAGE

// 5b
// 01

// PACKET

// 01 - type content
// 4b00 - data len
// be9f6f5e - unixtime (reversed)

// TAG DATA

// 970f270000
// c8049b4616
// 62049bb208
// c900000063
// 014502708d
// 0601010000
// be3f000719
// 3a02000000
// be423f0000
// 35ebcf0200
// be3d000000
// be44000000
// be3e3f0000
// 6332820002
// fa32010000

// END TAG DATA

// f5 - checksum

// END PACKET DATA

// 5d

// END PACKAGE

// 5b
// 01
// 01
// 4b00
// be48a667 67a648be (correct!)
// 970f270000
// c8049b4616 - sim
// 62049bb208 - full mileage
// c900000063
// 014602708d
// 0601010000
// be3f000719
// 3a02000000be423f000035ebcf0200be3d000000be44000000be3e3f00006332420002fa320100009f014b00fa48a667970f270000c8049b461662049bb208c9000000630145027d8d0601010000be3f0007193a02000000be423f000035ebcf0200be3d000000be44000000be3e3f00006332020002fa32010000a7014b003649a667970f270000c8049b461662049bb208c900000063014502628d0601010000be3f0007193a02000000be423f000035ebcf0200be3d000000be44000000be3e3f00006332020002fa32010000c95d000000000000000000000

// START PACKAGE

// 5b
// 01

// ------------

// START PACKETS

// 01 - data type
// 1e00 - data len
// ff66a667 - ts
// fbe200000cfd3095cf29fecc002d10caa9d2c829cb048ee20dffcb810600 - tagdata
// ef - checksum

// 01
// 4b00
// ff66a667
// 970f270000c8030d461662030db308c900000063014b021d8d0601010000be3f0007193a02000000be423e000035ebcf0200be3d000000be44000000be3e3e00006332220002fa08020000
// 48

// 01
// 4b00
// 3b67a667
// 970f270000c8030d461662030db308c90000006301ef0f1d8d0601010000be3f0007193a02000000be423e000035ebcf0200be3d000000be44000000be3e3e00006332020002fa32010000
// 3f

// 01
// 4b00
// 7767a667
// 970f270000c8030d461662030db308c90000006301f50f2b8d0601010000be3f0007193a02000000be423e000035ebcf0200be3d000000be44000000be3e3e00006332020002fa32010000
// 8f

// 01
// 4b00
// b367a667
// 970f270000c8030d461662030db308c9000000630183102b8d0601010000be3f0007193a02000000be423e000035ebcf0200be3d000000be44000000be3e3e00006332020002fa32010000
// 5a

// 01
// 4b00
// ef67a667
// 970f270000c8030d461662030db308c90000006301c80f1d8d0601010000be3f0007193a02000000be423e000035ebcf0200be3d000000be44000000be3e3e00006332020002fa32010000
// cc

// OTHER PACKETS DATA
// 014b002b68a667970f270000c8030d461662030db308c90000006301d80f108d0601010000be3f0007193a02000000be423e000035ebcf0200be3d000000be44000000be3e3e00006332020002fa320100000c014b006768a667970f270000c8030d461662030db308c90000006301f50f1d8d0601010000be3f0007193a02000000be423e000035ebcf0200be3d000000be44000000be3e3e00006332020002fa3201000072014b00a368a667970f270000c8030d461662030db308c90000006301a3101d8d0601010000be3f0007193a02000000be423e000035ebcf0200be3d000000be44000000be3e3e00006332020002fa320100005d014b00df68a667970f270000c8030d461662030db308c90000006301e60f108d0601010000be3f0007193a02000000be423e000035ebcf0200be3d000000be44000000be3e3e00006332020002fa32010000ce014b001b69a667970f270000c8030d461662030db308c90000006301ec0f1d8d0601010000be3f0007193a02000000be423e000035ebcf0200be3d000000be44000000be3e3e00006332020002fa320100001e5

// END PACKAGE
