package rabbit

import (
	"fmt"
	"log"
	"os"

	amqp "github.com/rabbitmq/amqp091-go"
)

func Conn() *amqp.Connection {
	usr, _ := os.LookupEnv("RMQ_USER")
	pass, _ := os.LookupEnv("RMQ_PASS")
	connStr := fmt.Sprintf("amqp://%s:%s@localhost:5672/", usr, pass)
	conn, err := amqp.Dial(connStr)
	if err != nil {
		log.Fatalf("Не удалось подключиться к RabbitMQ: %s", err)
	}

	return conn
}

func DeclareQueue(ch *amqp.Channel, queueName string) {
	// Объявляем очередь для команд
	_, err := ch.QueueDeclare(
		queueName, // Имя очереди
		false,     // Durable
		false,     // Delete when unused
		false,     // Exclusive
		false,     // NoWait
		nil,       // Arguments
	)
	if err != nil {
		log.Fatalf("Не удалось создать очередь: %s", err)
	}
}

func Consume(ch *amqp.Channel, queueName string) <-chan amqp.Delivery {
	msgs, err := ch.Consume(
		"arnavi_commands", // Имя очереди
		"",                // Consumer
		false,             // AutoAck
		false,             // Exclusive
		false,             // NoLocal
		false,             // NoWait
		nil,               // Arguments
	)
	if err != nil {
		log.Fatalf("Не удалось подписаться на очередь: %s", err)
	}

	return msgs
}
