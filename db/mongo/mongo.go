package mongo

import (
	"context"
	"fmt"
	"log"

	"go.mongodb.org/mongo-driver/mongo"
	"go.mongodb.org/mongo-driver/mongo/options"
)

func Connect(ctx context.Context, connStr string) (*mongo.Client, error) {
	clientOptions := options.Client().ApplyURI(connStr)
	client, err := mongo.Connect(ctx, clientOptions)
	if err != nil {
		defer func() {
			e := client.Disconnect(ctx)
			if e != nil {
				fmt.Println("Client disconnect err")
			}
		}()
		return nil, fmt.Errorf("Connect to MongoDB error with credentials:\n%v\n", connStr)
	}

	return client, nil
}

func GetAll(ctx context.Context, col *mongo.Collection) {
	curr, err := col.Find(ctx, nil)
	if err != nil {
		log.Fatalln(err.Error())
	}
	var results []any
	if err = curr.All(context.TODO(), &results); err != nil {
		panic(err)
	}
	fmt.Println(results...)
}

func Insert(ctx context.Context, col *mongo.Collection) {
	// col.InsertOne(ctx, )
}
