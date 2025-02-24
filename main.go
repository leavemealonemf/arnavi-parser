package main

import (
	"arnaviparser/broker/rabbit"
	. "arnaviparser/structs"
	"context"
	"database/sql"
	"encoding/hex"
	"encoding/json"
	"fmt"
	"io"
	"log"
	"math"
	"net"
	"net/http"
	"os"
	"strconv"
	"strings"
	"time"

	mg "arnaviparser/db/mongo"
	"arnaviparser/db/mongo/pg"

	. "arnaviparser/common/utils"

	. "arnaviparser/protocols/arnavi"

	"github.com/gorilla/mux"
	"github.com/joho/godotenv"
	_ "github.com/lib/pq"
	amqp "github.com/rabbitmq/amqp091-go"
	"github.com/rs/cors"
	"go.mongodb.org/mongo-driver/bson"
	"go.mongodb.org/mongo-driver/mongo"
	"go.mongodb.org/mongo-driver/mongo/options"
)

var devices []*Device

var connections map[int64]*Connection
var deviceStatusBitPos [][]int
var deviceIdsBytesAssotiation map[int]string
var commands map[string]*Command
var receivedCommands map[string]*ReceivedCommand
var addedImeis []int64
var db *sql.DB

const (
	tcpMsgBuff = 5000
)

func handleServe(conn net.Conn) {
	isFirstConn := true

	buff := make([]byte, tcpMsgBuff)

	var mainDevice *Device = &Device{}

	connection := &Connection{
		Conn:   conn,
		Device: mainDevice,
	}

	defer AbortTCPDeviceConn(connection)

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

			decIMEI := ParseIMEI(header.IMEI)
			stringImei := strconv.Itoa(int(decIMEI))
			// isFindImei := false

			// // VALIDATE IMEI IN DATABASE
			var rowImei string

			err := db.QueryRow("SELECT \"deviceIMEI\" FROM \"Scooter\" WHERE \"deviceIMEI\" = $1", "arnavi3:"+stringImei).Scan(&rowImei)
			if err != nil {
				fmt.Println("query row err:", err.Error())
				break
			}

			fmt.Println("In db. Continue...")

			// // END VALIDATE IMEI IN DATABASE

			mainDevice.IMEI = decIMEI

			connections[decIMEI] = connection

			// send SERVER_COM
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

			var totalPackets []*Device

			for {
				hexPacket := &HEXPacket{
					TypeOfContent: hexPackageData[start : start+2],
				}

				if strings.ToLower(hexPacket.TypeOfContent) == "01" {
					var device Device

					start += 2 // skip type
					hexPacket.PacketDataLen = hexPackageData[start : start+2]
					dataLenBytes := (HexToDec(hexPackageData[start:start+2]) * 2)
					start += 4 // skip len

					hexPacket.Unixtime = hexPackageData[start : start+8]
					timestamp := HexToDec(BytesToHexString(ReverseBytes(hexPacket.Unixtime))) * 1000
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

						tagIDDec := HexToDec(string(hexPacket.TagsData[i : i+2]))
						// tagFull := hexPacket.TagsData[i : i+10]
						tagParam := hexPacket.TagsData[i+2 : i+10]

						switch tagIDDec {
						// vs sensors
						case 190:
							internalTagIDDec := HexToDec(string(hexPacket.TagsData[i+2 : i+4]))
							internalTagParamRv := BytesToHexString(ReverseBytes(hexPacket.TagsData[i+4 : i+10]))
							switch internalTagIDDec {
							case 61:
								speedKmh := HexToDec(internalTagParamRv)
								device.VirtualSensors.SpeedKMH = uint8(speedKmh)
								break
							case 62:
								avgBtCharge := HexToDec(internalTagParamRv)
								device.VirtualSensors.AverageBatteryCharge = uint8(avgBtCharge)
								break
							case 63:
								internalParam := hexPacket.TagsData[i+4 : i+10]
								DecodeVSStatementFlags(internalParam, &device)
								break
							case 66:
								mainBtCharge := HexToDec(internalTagParamRv)
								device.VirtualSensors.MainBatteryCharge = uint8(mainBtCharge)
							case 67:
								additionalBtCharge := HexToDec(internalTagParamRv)
								device.VirtualSensors.AdditionalBatteryCharge = uint8(additionalBtCharge)
							case 68:
								errCode := HexToDec(internalTagParamRv)
								device.VirtualSensors.ErrorCode = uint8(errCode)
							case 69:
								mileagePerTrip := HexToDec(internalTagParamRv)
								device.VirtualSensors.MileagePerTrip = uint32(mileagePerTrip)
							case 70:
								motorWheelControllerErrors := HexToDec(internalTagParamRv)
								device.VirtualSensors.MotorWheelControllerErrors = uint16(motorWheelControllerErrors)
							case 71:
								bmsError := HexToDec(internalTagParamRv)
								device.VirtualSensors.BMSErrors = uint8(bmsError)
							default:
								break
							}
							break
						// device status
						case 99:
							tagParamRv := BytesToHexString(ReverseBytes(tagParam))
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
							tagParamRv := BytesToHexString(ReverseBytes(tagParam))
							ParseTAG9(tagParamRv, &device)
							break
						case 6:
							param := tagParam[2:len(tagParam)]
							p := BytesToHexString(ReverseBytes(param))
							ParseTAG6(p, &device)
							break
						case 3, 4:
							rvParamNum := HexToDec(BytesToHexString(ReverseBytes(tagParam)))
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
							ParseTAG1Data(ReverseBytes(tagParam), &device)
							break
						case 7, 200:
							if tagIDDec == 7 {
								ParseTags7And200(ReverseBytes(tagParam), &device, 1)
							} else {
								ParseTags7And200(ReverseBytes(tagParam), &device, 2)
							}
							break
						case 8, 201:
							if tagIDDec == 8 {
								ParseTags8And201(ReverseBytes(tagParam), &device, 1)
							} else {
								ParseTags8And201(ReverseBytes(tagParam), &device, 2)
							}
							break

						// case 253, 254, 202, 230:
						// 	if tagIDDec == 253 || tagIDDec == 202 {
						// 		processICCID(ReverseBytes(tagParam), &device, 1)
						// 	} else {
						// 		processICCID(ReverseBytes(tagParam), &device, 2)
						// 	}
						// 	break
						default:
							break
						}

						// fmt.Printf("decimal tag_id: %v\nfull hex_tag: %v\ntag_param_without_id: %v\n", tagIDDec, tagFull, tagParam)
						// fmt.Println("--------------------")

					}

					// printHexPacketStructData(hexPacket)

					totalPackets = append(totalPackets, &device)

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

					receivedCommand := receivedCommands[token]

					if receivedCommand == nil {
						break
					}

					if receivedCommand.Status == "completed" || receivedCommand.Status == "error" {
						break
					}

					if strings.Compare(errCode, "00") == 0 {
						receivedCommand.Status = "completed"
					} else {
						receivedCommand.Status = "error"
					}

					f := bson.D{{Key: "token", Value: token}}

					upd := bson.D{
						{"$set", bson.D{
							{Key: "status", Value: receivedCommand.Status},
							{Key: "_ct", Value: time.Now().UnixMicro()},
						}},
					}
					mg.UpdOne(ctx, cmdsColl, f, upd)
					fmt.Println(pktType, errCode, token, cs)
					receivedCommand.ExecChannel <- true
					// delete(receivedCommands, token)

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

			fmt.Println("total packet's in package:", len(totalPackets))
			fmt.Println("Device IMEI:", mainDevice.IMEI)
			fmt.Println("print json info about single packet:")
			for i := 0; i < len(totalPackets); i++ {
				if totalPackets[i] != nil {
					fmt.Println("------------------------")
					fmt.Println("Packet", i+1, ":")
					pck, _ := json.Marshal(totalPackets[i])
					fmt.Println(string(pck))
				}
			}

			for i := 0; i < len(totalPackets); i++ {
				if totalPackets[i] != nil {
					if (i + 1) == len(totalPackets) {
						mainDevice = totalPackets[i]
						break
					}
					if totalPackets[i].Lat != 0 && totalPackets[i].Lon != 0 {
						mainDevice = totalPackets[i]
						break
					}
				}
			}

			mainDevice.ServerTime = time.Now().UnixMicro()
			BindDeviceMainPropertys(mainDevice)
			mainDevice.Online = true
			marshal, _ := json.Marshal(mainDevice)
			publishPacket(marshal)
			_, e := scooterColl.InsertOne(ctx, mainDevice)
			if e != nil {
				fmt.Println("Insert scooter error")
			} else {
				fmt.Println("Insert scooter successfully")
			}
		}
	}
}

func BindDeviceMainPropertys(device *Device) {
	device.Charge = device.VirtualSensors.MainBatteryCharge
	device.Speed = device.VirtualSensors.SpeedKMH
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

	if device.DeviceStatus1 != nil {
		// tag_9
		device.MoveSensor = device.DeviceStatus1["mw"] == 1
	} else {
		// tag_99
		device.MoveSensor = device.DeviceStatus2["mw"] == 1
	}

	if device.TagSix != nil {
		device.Alarm = device.TagSix["alarm_st"] == 1
	}

	device.AverageCharge = device.VirtualSensors.AverageBatteryCharge
	device.AdditionalCharge = device.VirtualSensors.AdditionalBatteryCharge
	device.DriveMode = device.VirtualSensors.StatementFlags.Mode
	device.VsErrCode = device.VirtualSensors.ErrorCode
	device.ScreenOff = device.VirtualSensors.StatementFlags.ScreenOff
	device.Ignition = device.VirtualSensors.StatementFlags.MotorRunning
}

func sendServerComSuccessed(codeLine string, conn net.Conn) {
	fmt.Printf("[LINE %v] Get package. Sending SERVER_COM success...\n", codeLine)
	sComPackage, _ := hex.DecodeString("7B00017D")
	conn.Write(sComPackage)
}

func sendServerComFailed(codeLine string, conn net.Conn) {
	fmt.Printf("[LINE %v] Get package. Sending SERVER_COM failed...\n", codeLine)
	sComPackage, _ := hex.DecodeString("7B00FE7D")
	conn.Write(sComPackage)
}

func AbortTCPDeviceConn(conn *Connection) {
	if connections[conn.Device.IMEI] != nil {
		filter := bson.D{
			{Key: "imei", Value: conn.Device.IMEI},
		}

		update := bson.D{
			{"$set", bson.D{
				{Key: "online", Value: false},
			}},
		}

		opts := options.FindOneAndUpdate().SetSort(bson.D{{Key: "_ts", Value: -1}})
		mg.UpdOneScooter(ctx, scooterColl, filter, update, opts)

		conn.Conn.Close()

		delete(connections, conn.Device.IMEI)
	}
}

func processICCID(revBytes []byte, device *Device, partNum uint8) {
	device.ICCIDParts[partNum] = revBytes
}

func printHexPacketStructData(packet *HEXPacket) {
	fmt.Printf("type of content: %v\ndata len: %v\npacket unixtime: %v\npacket tags data: %v\nchecksum: %v\n\n", packet.TypeOfContent, packet.PacketDataLen, packet.Unixtime, packet.TagsData, packet.Checksum)
}

func initIOTCommands() {
	commands = map[string]*Command{}
	commands["engine-m1txq"] = &Command{
		Val:    "51330001",
		NameEn: "Block Scooter Motor Wheel",
		NameRu: "Блокировка мотор колеса",
	}
	commands["unlock-tt3rm"] = &Command{
		Val:    "51330000",
		NameEn: "Disable Guard Mode",
		NameRu: "Снять с режима охраны",
	}
	commands["lock-lyoqv"] = &Command{
		Val:    "51330005",
		NameEn: "Enable Guard Mode",
		NameRu: "Установить режим охраны",
	}
	commands["51330006"] = &Command{
		Val:    "51330006",
		NameEn: "Set Service Mode",
		NameRu: "Установить режим сервиса",
	}
	commands["speed-mode-4i6ku"] = &Command{
		Val:    "51330300",
		NameEn: "Set Drive Mode D",
		NameRu: "Установить стиль езды D",
	}
	commands["speed-mode-72fcz"] = &Command{
		Val:    "51330301",
		NameEn: "Set Drive Mode ECO",
		NameRu: "Установить стиль езды ECO",
	}
	commands["speed-mode-uohen"] = &Command{
		Val:    "51330302",
		NameEn: "Set Drive Mode S",
		NameRu: "Установить стиль езды S",
	}
	commands["lamp-xurbd"] = &Command{
		Val:    "51080201",
		NameEn: "Turn On Flash",
		NameRu: "Включить фонарь",
	}
	commands["lamp-k2uex-umofv-0hpjw"] = &Command{
		Val:    "51080200",
		NameEn: "Turn Off Flash",
		NameRu: "Выключить фонарь",
	}
	// speed
	commands["speed-limit-norm-jxl0w"] = &Command{
		Val:    "51330405",
		NameEn: "Speed Limit 5 km/h",
		NameRu: "Ограничение скорости 5 км/ч",
	}
	commands["speed-limit-norm-cd8mx"] = &Command{
		Val:    "5133040A",
		NameEn: "Speed Limit 10 km/h",
		NameRu: "Ограничение скорости 10 км/ч",
	}
	commands["speed-limit-norm-7s2eo"] = &Command{
		Val:    "5133040F",
		NameEn: "Speed Limit 15 km/h",
		NameRu: "Ограничение скорости 15 км/ч",
	}
	commands["speed-limit-norm-ldzq6"] = &Command{
		Val:    "51330414",
		NameEn: "Speed Limit 20 km/h",
		NameRu: "Ограничение скорости 20 км/ч",
	}
	commands["speed-limit-norm-gyjf3"] = &Command{
		Val:    "51330419",
		NameEn: "Speed Limit 25 km/h",
		NameRu: "Ограничение скорости 25 км/ч",
	}

	commands["beeper-j5shv"] = &Command{
		Val:    "1803000A",
		NameEn: "Найти самокат",
		NameRu: "Найти самокат",
	}

	commands["unlock-wheel-lock1"] = &Command{
		Val:    "51080001",
		NameEn: "Открыть замок 1 на 4 секунды",
		NameRu: "Открыть замок 1 на 4 секунды",
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

func HTTPCommandHandler(w http.ResponseWriter, r *http.Request) {
	if r.Method == "POST" {
		body, err := io.ReadAll(r.Body)

		if err != nil {
			http.Error(w, "failed to read bytes", 400)
		}

		var cmdS QueueCmd
		err = json.Unmarshal(body, &cmdS)
		if err != nil {
			http.Error(w, "Failed to parse JSON response", http.StatusInternalServerError)
			return
		}

		resStr := strings.Split(cmdS.ImeiWithPrefix, ":")
		cmd := cmdS.CMD
		imei := resStr[1]
		decImei, _ := strconv.Atoi(imei)

		c := connections[int64(decImei)]
		if c != nil {
			cmdInfo := commands[cmd]

			if cmdInfo == nil {
				w.WriteHeader(404)
				w.Write([]byte("this command does not exist"))
				return
			}

			token, _ := GenСmdTokenHex()
			tokenBy := HexToBytes(token)
			cmdBy := HexToBytes(cmdInfo.Val)

			totalBy := make([]byte, 2)
			totalBy = append(totalBy, tokenBy...)
			totalBy = append(totalBy, cmdBy...)

			cs := Checksum(totalBy)

			command := fmt.Sprintf("7B08FF%s%s%s7D", cs, token, cmdInfo.Val)
			sComPackage, _ := hex.DecodeString(command)
			cmdChan := make(chan bool)

			recievedCmd := &ReceivedCommand{
				ServerTime:  time.Now().UnixMicro(),
				CMD:         command,
				Token:       token,
				Status:      "pending",
				IMEI:        imei,
				CMDInfo:     commands[cmd],
				ExecChannel: cmdChan,
			}

			receivedCommands[token] = recievedCmd
			mg.Insert(ctx, cmdsColl, recievedCmd)
			c.Conn.Write(sComPackage)

			select {
			case success := <-cmdChan:
				if success {
					w.Write([]byte(fmt.Sprintf("Command %s executed successfully", cmd)))
					delete(receivedCommands, token)
				} else {
					http.Error(w, "Command execution failed", http.StatusInternalServerError)
					delete(receivedCommands, token)
				}
			case <-time.After(60 * time.Second):
				http.Error(w, "Command execution timed out", http.StatusRequestTimeout)
				delete(receivedCommands, token)
			}

		} else {
			w.WriteHeader(404)
			w.Write([]byte("device not connected"))
		}
	}

}

func initHttp() {
	r := mux.NewRouter()

	r.HandleFunc("/cmd", HTTPCommandHandler)

	addr := fmt.Sprintf("0.0.0.0:%d", 9921)

	c := cors.New(cors.Options{
		AllowedOrigins:   []string{"*"},
		AllowCredentials: true,
	})

	handler := c.Handler(r)

	srv := &http.Server{
		Handler:      handler,
		Addr:         addr,
		WriteTimeout: 60 * time.Second,
		ReadTimeout:  60 * time.Second,
	}

	fmt.Printf("http server up on :%d\n", 9921)
	log.Fatal(srv.ListenAndServe())
}

var rbtConn *amqp.Connection
var rbtCh *amqp.Channel

func publishPacket(pkt []byte) {
	err := rbtCh.Publish(
		"",
		"packets",
		false,
		false,
		amqp.Publishing{
			ContentType: "application/json",
			Body:        pkt,
		},
	)
	if err != nil {
		log.Println("failed to publish message", err.Error())
	}
}

func showConnectedDevices() {
	for {
		fmt.Println("-----------------------------------------")
		connCount := len(connections)
		fmt.Println("Connected device count:", connCount)
		if connCount == 0 {
			fmt.Println("-----------------------------------------")
			time.Sleep(time.Second * 30)
			continue
		}

		fmt.Println("Connected IMEI's:")
		for key, value := range connections {
			fmt.Println("Key:", key, "IMEI device struct value:", value.Device.IMEI)
		}
		fmt.Println("-----------------------------------------")
		time.Sleep(time.Second * 30)
	}
}

func main() {
	db = pg.ConnectPG()
	defer db.Close()

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

	go initHttp()

	serve, err := net.Listen("tcp", ":20550")

	devices = make([]*Device, 0)
	connections = map[int64]*Connection{}
	deviceStatusBitPos = [][]int{{27, 26}, {25}, {24, 23}, {22}, {21, 20}, {19, 18}, {17, 16}, {15, 14}, {13, 12}, {11, 10, 9}, {8, 7, 6}, {5}, {4, 3, 2}, {1, 0}}
	deviceIdsBytesAssotiation = map[int]string{
		0: "device_status", 1: "bt", 2: "msd", 3: "guard_zone_ctrl", 4: "mw", 5: "s3_st", 6: "s2_st", 7: "s1_st", 8: "s0_st", 9: "sim2_st", 10: "sim1_st", 11: "sim_t", 12: "gsm_st", 13: "nav_st",
	}
	addedImeis = append(addedImeis, 866011050296805, 866039048453774)
	receivedCommands = map[string]*ReceivedCommand{}

	initIOTCommands()

	if err != nil {
		log.Fatalln("Startup serve error:", err.Error())
	}

	log.Println("Server started:", serve.Addr().Network())

	rbtConn = rabbit.Conn()
	ch, err := rbtConn.Channel()
	rbtCh = ch

	if err != nil {
		log.Fatalln(err.Error())
	}

	defer ch.Close()

	_, err = ch.QueueDeclare(
		"packets",
		false,
		false,
		false,
		false,
		nil,
	)

	go showConnectedDevices()

	for {
		conn, err := serve.Accept()

		if err != nil {
			log.Fatalln("accept connection error:", err.Error())
		}

		fmt.Printf("Received new connection:\n%v\n%v\n\r", conn.RemoteAddr().String(), conn.LocalAddr().String())

		go handleServe(conn)
	}
}
