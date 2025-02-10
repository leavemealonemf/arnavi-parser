package main

import (
	"encoding/hex"
	"fmt"
	"log"
	"strconv"
)

func BytesToHexString(bytes []byte) string {
	encoded := hex.EncodeToString(bytes)
	return encoded
}

func parseIMEI(hexIMEI string) (string, int64) {
	bytes, _ := hex.DecodeString(hexIMEI)

	for i, j := 0, len(bytes)-1; i < j; i, j = i+1, j-1 {
		bytes[i], bytes[j] = bytes[j], bytes[i]
	}

	reversedIMEI := BytesToHexString(bytes)

	decIMEI, _ := strconv.ParseInt(reversedIMEI, 16, 64)
	return reversedIMEI, decIMEI
}

func reverseBytes(hexString string) []byte {
	bytes, _ := hex.DecodeString(hexString)

	for i, j := 0, len(bytes)-1; i < j; i, j = i+1, j-1 {
		bytes[i], bytes[j] = bytes[j], bytes[i]
	}

	return bytes
}

func PacketHexChecksum(h string) string {
	var packetData string = h
	revBytes := reverseBytes(packetData)

	checksum := byte(0)
	for _, b := range revBytes {
		checksum += b
	}

	return fmt.Sprintf("%02x", checksum)
}

func HexToStr() {
	a := "0e000232"
	bs, err := hex.DecodeString(a)
	if err != nil {
		panic(err)
	}
	fmt.Println(string(bs))
}

func main() {
	// headerHex := "ff23e54506e9a11303000000000000000000000000000000000000000000000"
	// notParsedImei := "e54506e9a1130300"
	// fmt.Println(headerHex[0:2])
	// fmt.Println(headerHex[2:4])
	// fmt.Println(headerHex[4:20])

	// parseIMEI(notParsedImei)
	// hexPacket := "5b01011e00ff66a667fbe200000cfd3095cf29fecc002d10caa9d2c829cb048ee20dffcb810600ef014b00ff66a667970f270000c8030d461662030db308c900000063014b021d8d0601010000be3f0007193a02000000be423e000035ebcf0200be3d000000be44000000be3e3e00006332220002fa0802000048014b003b67a667970f270000c8030d461662030db308c90000006301ef0f1d8d0601010000be3f0007193a02000000be423e000035ebcf0200be3d000000be44000000be3e3e00006332020002fa320100003f014b007767a667970f270000c8030d461662030db308c90000006301f50f2b8d0601010000be3f0007193a02000000be423e000035ebcf0200be3d000000be44000000be3e3e00006332020002fa320100008f014b00b367a667970f270000c8030d461662030db308c9000000630183102b8d0601010000be3f0007193a02000000be423e000035ebcf0200be3d000000be44000000be3e3e00006332020002fa320100005a014b00ef67a667970f270000c8030d461662030db308c90000006301c80f1d8d0601010000be3f0007193a02000000be423e000035ebcf0200be3d000000be44000000be3e3e00006332020002fa32010000cc014b002b68a667970f270000c8030d461662030db308c90000006301d80f108d0601010000be3f0007193a02000000be423e000035ebcf0200be3d000000be44000000be3e3e00006332020002fa320100000c014b006768a667970f270000c8030d461662030db308c90000006301f50f1d8d0601010000be3f0007193a02000000be423e000035ebcf0200be3d000000be44000000be3e3e00006332020002fa3201000072014b00a368a667970f270000c8030d461662030db308c90000006301a3101d8d0601010000be3f0007193a02000000be423e000035ebcf0200be3d000000be44000000be3e3e00006332020002fa320100005d014b00df68a667970f270000c8030d461662030db308c90000006301e60f108d0601010000be3f0007193a02000000be423e000035ebcf0200be3d000000be44000000be3e3e00006332020002fa32010000ce014b001b69a667970f270000c8030d461662030db308c90000006301ec0f1d8d0601010000be3f0007193a02000000be423e000035ebcf0200be3d000000be44000000be3e3e00006332020002fa320100001e5d"
	// revHex, _ := parseIMEI(hexPacket)
	// fmt.Println(revHex)
	// fmt.Println(PacketHexChecksum("0101"))
	// HexToStr()
	// num := uint32(0x0e000232)

	// Выводим число в двоичном виде
	// fmt.Printf("Число: %032b\n", num)

	// Разбираем биты по частям
	// num := int(0x0e000232)
	// deviceStatusBitPos := [][]int{{27, 26}, {25}, {24, 23}, {22}, {21, 20}, {19, 18}, {17, 16}, {15, 14}, {13, 12}, {11, 10, 9}, {8, 7, 6}, {5}, {4, 3, 2}, {1, 0}}

	// // deviceIdsBytesAssotiation := map[int]string{
	// // 	0: "device_status", 1: "bt", 2: "msd", 3: "guard_zone_ctrl", 4: "mw", 5: "S3_st", 6: "S2_st", 7: "S1_st", 8: "S0_st", 9: "sim2_st", 10: "sim1_st", 11: "sim_t", 12: "gsm_st", 13: "nav_st",
	// // }

	// devicePreResult := map[int]int{}

	// num := int64(0x0a008232)

	// for i := 0; i < len(deviceStatusBitPos); i++ {
	// 	var result int64
	// 	for j := 0; j < len(deviceStatusBitPos[i]); j++ {
	// 		bit := (num >> deviceStatusBitPos[i][j]) & 1
	// 		result |= bit << (len(deviceStatusBitPos[i]) - j - 1)
	// 	}
	// 	devicePreResult[i] = int(result)
	// }

	// fmt.Println(devicePreResult)

	// for i := 0; i < len(devicePreResult); i++ {
	// 	assotiation := ids_bytes_assotiation[i]
	// 	fmt.Println(assotiation)
	// 	deviceResult[assotiation] = devicePreResult[i]
	// }

	// fmt.Printf("device_pre_result: %v\n", devicePreResult)
	// fmt.Printf("device_result: %v\n", deviceResult)

	// num2 := int64(0x0a008232)

	// for i := 31; i >= 0; i-- {
	// 	bit := (num2 >> i) & 1
	// 	fmt.Printf("Бит %2d: %d\n", i, bit)
	// }

	// 190700  000719

	// num := hexToDec("000719")
	// num := uint32(0x000719)

	decodeStatus("000b00")
	// newFlags := byte(0x19)
	// newType := byte((num >> 8) & 0xFF)
	// fmt.Printf("Флаги состояний: %08b\n", newFlags)
	// fmt.Printf("Тип устройства: %d\n", newType)

	// fmt.Println(PacketHexChecksum("67A8C24B"))
}

func hexToDec(hexString string) int64 {
	dec, _ := strconv.ParseInt(hexString, 16, 64)
	return dec
}

func decodeStatus(hexStr string) {
	data, err := hex.DecodeString(hexStr)
	if err != nil || len(data) < 2 {
		log.Fatalf("Invalid hex string: %s", hexStr)
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

	fmt.Printf("Motor Running: %v\n", motorRunning)
	fmt.Printf("Mode: %s\n", modes[mode])
	fmt.Printf("Charging: %v\n", charging)
	fmt.Printf("Screen Off/Charging Done: %v\n", screenOff)
	fmt.Printf("Pedestrian Mode: %v\n", pedestrianMode)
	fmt.Printf("Overheat: %v\n", overheat)
	fmt.Printf("Scooter Type: %d\n", byte1)
}

// type of content: 01
// data len: 1e
// packet unixtime: ff66a667
// packet tags data: fbe200000cfd3095cf29fecc002d10caa9d2c829cb048ee20dffcb810600
// checksum: ef
