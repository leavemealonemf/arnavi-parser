package mongo

import (
	"context"
	"fmt"
	"log"

	"go.mongodb.org/mongo-driver/mongo"
)

func Seed(client *mongo.Client, ctx context.Context) {
	scootColl := client.Database("local").Collection("scooters")
	if scootColl == nil {
		err := client.Database("local").CreateCollection(ctx, "scooters")
		if err != nil {
			log.Fatalln("Failed to create mongo scooter coolection", err.Error())
		}
	} else {
		fmt.Println("Mongo scooter coll already exist")
	}
	cmdsColl := client.Database("local").Collection("cmds")
	if cmdsColl == nil {
		err := client.Database("local").CreateCollection(ctx, "cmds")
		if err != nil {
			log.Fatalln("Failed to create mongo commands coolection", err.Error())
		}
	} else {
		fmt.Println("Mongo commands coll already exist")
	}
	fmt.Println("[MONGO] successfull seed.")
}
