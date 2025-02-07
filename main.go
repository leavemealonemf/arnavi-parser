package main

import (
	"encoding/hex"
	"fmt"
	"log"
	"net"
)

func handleServe(conn net.Conn) {
	defer conn.Close()

	isFirstConn := true

	buff := make([]byte, 1024)

	for {
		_, err := conn.Read(buff)
		if err != nil {
			fmt.Println("Received data err:", err.Error())
			break
		}
		encodedString := hex.EncodeToString(buff)
		fmt.Println("Received msg:", encodedString)

		// msg, err := bufio.NewReader(conn).ReadString('\n')
		// if err != nil {
		// 	fmt.Println("Received data err:", err.Error())
		// 	break
		// }

		if isFirstConn {
			data, err := hex.DecodeString("7B0400CA5E9F6F5E7D")

			if err != nil {
				panic(err)
			}

			conn.Write(data)
			fmt.Println("sending server com...")
			isFirstConn = false
		}
	}
}

func main() {
	serve, err := net.Listen("tcp", ":20550")

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

// 6f5e970f270000c8049b461662049bb208c90000

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

// PACKET

// f5 - checksum

// 5d

// 5b
// 01
// 01
// 4b00
// be48a667 67a648be (correct!)
// 970f270000c8049b461662049bb208c900000063014602708d0601010000be3f0007193a02000000be423f000035ebcf0200be3d000000be44000000be3e3f00006332420002fa320100009f014b00fa48a667970f270000c8049b461662049bb208c9000000630145027d8d0601010000be3f0007193a02000000be423f000035ebcf0200be3d000000be44000000be3e3f00006332020002fa32010000a7014b003649a667970f270000c8049b461662049bb208c900000063014502628d0601010000be3f0007193a02000000be423f000035ebcf0200be3d000000be44000000be3e3f00006332020002fa32010000c95d000000000000000000000
