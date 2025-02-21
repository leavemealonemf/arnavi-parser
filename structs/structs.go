package structs

import (
	"net"

	amqp "github.com/rabbitmq/amqp091-go"
)

type Connection struct {
	Conn   net.Conn
	Device *Device
}

type IOTResponseJournal struct {
	ID         int64             `json:"id" bson:"id"`
	State      *Device           `json:"state" bson:"state"`
	CMDJournal []ReceivedCommand `json:"cmd_journal" bson:"cmd_journal"`
}

type IOTResponse struct {
	ID    int64   `json:"id" bson:"id"`
	State *Device `json:"state" bson:"state"`
}

type ReceivedCommand struct {
	ServerTime    int64         `json:"_ts" bson:"_ts"`
	CompletedTime int64         `json:"_ct" bson:"_ct"`
	CMD           string        `json:"hex_origin" bson:"hex_origin,omitempty"`
	Token         string        `json:"token" bson:"token,omitempty"`
	Status        string        `json:"status" bson:"status,omitempty"`
	IMEI          string        `json:"dvce_imei" bson:"dvce_imei,omitempty"`
	CMDInfo       *Command      `json:"cmd_info" bson:"cmd_info"`
	QueueD        amqp.Delivery `json:"-" bson:"-"`
	ExecChannel   chan bool     `json:"-" bson:"-"`
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
	MotorRunning   bool  `json:"motor_running" bson:"motor_running,omitempty"`
	Mode           byte  `json:"mode" bson:"mode,omitempty"`
	Charging       bool  `json:"charging" bson:"charging,omitempty"`
	ScreenOff      bool  `json:"screen_off" bson:"screen_off,omitempty"`
	PedestrianMode bool  `json:"pedestrian_mode" bson:"pedestrian_mode,omitempty"`
	Overheat       bool  `json:"overheat" bson:"overheat,omitempty"`
	ScooterType    uint8 `json:"scooter_type" bson:"scooter_type,omitempty"`
}

type DeviceVS struct {
	SpeedKMH                   uint8                  `json:"speed_kmh" bson:"speed_kmh,omitempty"`
	AverageBatteryCharge       uint8                  `json:"avg_battery_charge" bson:"avg_battery_charge,omitempty"`
	MainBatteryCharge          uint8                  `json:"main_battery_charge" bson:"main_battery_charge,omitempty"`
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
	ServerTime       int64             `json:"_ts" bson:"_ts,omitempty"`
	Timestamp        int64             `json:"time" bson:"time,omitempty"`
	Online           bool              `json:"online" bson:"online,omitempty"`
	IMEI             int64             `json:"imei" bson:"imei,omitempty"`
	Speed            uint8             `json:"speed_kmh" bson:"speed_kmh,omitempty"`       // vs_64 aka self.VirtualSensors.SpeedKMH (binded)
	Charge           uint8             `json:"charge" bson:"charge,omitempty"`             // vs_64 aka self.VirtualSensors.MainBatteryCharge (binded)
	SpeedKnots       float64           `json:"speed" bson:"speed,omitempty"`               // tag_5.speed (binded)
	Alt              int               `json:"altitude" bson:"altitude,omitempty"`         // tag_5.altitude (binded)
	Azimut           int               `json:"azimut" bson:"azimut,omitempty"`             // tag_5.azimut (binded)
	Lat              float32           `json:"lat" bson:"lat,omitempty"`                   // tag_3 (binded)
	Lon              float32           `json:"lon" bson:"lon,omitempty"`                   // tag_4 (binded)
	IsSim            bool              `json:"isSim" bson:"isSim,omitempty"`               // tag_99 [sim_1st] && [sim_2st] aka self.DeviceStatus["sim_1st"] && self.DeviceStatus["sim_2st"] (binded)
	SimNumber        uint8             `json:"simNumber" bson:"simNumber,omitempty"`       // tag_99 [sim_t] aka self.DeviceStatus["sim_t"] (binded)
	MoveSensor       bool              `json:"mover_sensor" bson:"mover_sensor,omitempty"` // tag_99 [mv] aka self.DeviceStatus["mv"] (binded)
	SatGps           uint8             `json:"sat_gps" bson:"sat_gps,omitempty"`           // tag_5 (binded)
	SatGlonass       uint8             `json:"sat_glonass" bson:"sat_glonass,omitempty"`   // tag_5 (binded)
	TotalSatellites  byte              `json:"total_sat" bson:"total_sat,omitempty"`       // tag_5 (binded)
	GPS              uint8             `json:"gps" bson:"gps,omitempty"`                   // tag_99 [nav_st] aka self.DeviceStatus["nav_st"] (binded)
	GSM              uint8             `json:"gsm" bson:"gsm,omitempty"`                   // tag_99 [gsm_st] aka self.DeviceStatus["gsm_st"] (binded)
	LockStatus       bool              `json:"lock_status" bson:"lock_status,omitempty"`   // tag_99 [device_status] aka self.DeviceStatus["device_status"] (binded)
	Charging         bool              `json:"charging" bson:"charging,omitempty"`         // vs_63 [device_status] aka self.VirtualSensors.StatementFlags.Charging (binded)
	Mnc              uint32            `json:"mnc" bson:"mnc,omitempty"`                   // tag_7 cellID
	DeviceStatus2    map[string]int    `json:"tag_99" bson:"tag_99,omitempty"`
	DeviceStatus1    map[string]uint32 `json:"tag_9" bson:"tag_9,omitempty"`
	TagSix           map[string]int64  `json:"tag_6" bson:"tag_6,omitempty"`
	TagFive          TAGFive           `json:"tag_5" bson:"tag_5,omitempty"`
	TAGOne           TAGOne            `json:"tag_1" bson:"tag_1,omitempty"`
	SimOne           *SimStatus        `json:"sim_1" bson:"sim_1,omitempty"`
	SimTwo           *SimStatus        `json:"sim_2" bson:"sim_2,omitempty"`
	VirtualSensors   DeviceVS          `json:"vs" bson:"vs,omitempty"`
	ICCIDParts       map[uint8][]byte  `json:"iccid_parts,omitempty" bson:"iccid_parts,omitempty"`
	Alarm            bool              `json:"alarm" bson:"alarm"`
	AverageCharge    uint8             `json:"average_charge" bson:"average_charge"`
	AdditionalCharge uint8             `json:"additional_charge" bson:"additional_charge"`
	DriveMode        uint8             `json:"drive_mode" bson:"drive_mode"`
	VsErrCode        uint8             `json:"vs_err_code" bson:"vs_err_code"`
	Ignition         bool              `json:"ignition" bson:"ignition"`
	ScreenOff        bool              `json:"screen_off" bson:"screen_off"`
}

type Command struct {
	Val    string `json:"cmd_hex" bson:"cmd_hex,omitempty"`
	NameEn string `json:"name_en" bson:"name_en,omitempty"`
	NameRu string `json:"name_ru" bson:"name_ru,omitempty"`
}

type QueueCmd struct {
	CMD            string `json:"Cmd"`
	ImeiWithPrefix string `json:"IMEI"`
}
