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
			conn.Write([]byte("7B 05 C7 01 02 03 04 05 7D"))
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
