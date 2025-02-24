package arnavi

import (
	. "arnaviparser/common/utils"
	. "arnaviparser/structs"
	"crypto/rand"
	"encoding/hex"
	"fmt"
	"strconv"
)

func ParseIMEI(hexIMEI string) int64 {
	bytes, _ := hex.DecodeString(hexIMEI)

	for i, j := 0, len(bytes)-1; i < j; i, j = i+1, j-1 {
		bytes[i], bytes[j] = bytes[j], bytes[i]
	}

	reversedIMEI := BytesToHexString(bytes)

	decIMEI, _ := strconv.ParseInt(reversedIMEI, 16, 64)
	return decIMEI
}

func PacketHexChecksum(hexPacket *HEXPacket) string {
	var packetData string = hexPacket.Unixtime + hexPacket.TagsData
	revBytes := ReverseBytes(packetData)

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
	bytes := make([]byte, 5)
	_, err := rand.Read(bytes)
	if err != nil {
		return "", fmt.Errorf("GenСmdTokenHex err: %v\n", err.Error())
	}
	bytes[0] = 0xFF
	return hex.EncodeToString(bytes), nil
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

	//modes := map[byte]string{
	//	0b00: "D",
	//	0b01: "ECO",
	//	0b10: "S",
	//}

	device.VirtualSensors.StatementFlags.MotorRunning = motorRunning
	device.VirtualSensors.StatementFlags.Mode = mode
	device.VirtualSensors.StatementFlags.Charging = charging
	device.VirtualSensors.StatementFlags.ScreenOff = screenOff
	device.VirtualSensors.StatementFlags.PedestrianMode = pedestrianMode
	device.VirtualSensors.StatementFlags.Overheat = overheat
	device.VirtualSensors.StatementFlags.ScooterType = byte1
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

func ParseTAG5Data(hexValue string, device *Device) {
	data := ReverseBytes(hexValue)

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
	value := HexToDec(hexStringRev)

	ignitionState := (value >> 0) & 0x01
	doorLock1State := (value >> 8) & 0x01
	doorLock2State := (value >> 9) & 0x01
	flashlightState := (value >> 16) & 0x01
	alarmState := (value >> 17) & 0x01
	usbPowerState := (value >> 18) & 0x01

	device.TagSix = map[string]int64{
		"ignition_st":      ignitionState,
		"door_one_lock_st": doorLock1State,
		"door_two_lock_st": doorLock2State,
		"flash_light_st":   flashlightState,
		"alarm_st":         alarmState,
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
